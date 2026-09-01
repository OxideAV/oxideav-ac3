//! Reference stereo renderer for reconstructed E-AC-3 JOC objects.
//!
//! ETSI TS 103 420 specifies how to recover object essences and their
//! rendering metadata, but deliberately leaves adaptation to a playback
//! speaker layout to the renderer. This module uses an equal-power stereo
//! policy: the room/screen X coordinate pans between L/R, object width
//! spreads a point source toward both speakers, and Y/Z remain available
//! for a future loudspeaker or binaural renderer.

use std::{f32::consts::FRAC_PI_2, fmt};

use super::{
    joc::JocMetadata,
    joc_qmf::{
        Complex32, QmfAnalysis, QmfSynthesis, QMF_RECONSTRUCTION_DELAY, QMF_SUBBANDS,
        QMF_TIMESLOTS_PER_EAC3_FRAME,
    },
    joc_reconstruct::{JocMatrixDecoder, JocReconstructionError},
    oamd::{OamdDecoder, OamdFrame, OamdObjectKind, OamdObjectState, OamdParseError},
};

const SAMPLES_PER_JOC_FRAME: usize = QMF_SUBBANDS * QMF_TIMESLOTS_PER_EAC3_FRAME;
const MAX_JOC_OBJECTS: usize = 16;
const LFE_STEREO_GAIN: f32 = 0.5;
const DIFFUSE_STEREO_GAIN: f32 = std::f32::consts::FRAC_1_SQRT_2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
// internal — exposed for tests/fuzz; not part of the stable API
#[doc(hidden)]
pub enum JocRenderError {
    Invalid(&'static str),
    Unsupported(&'static str),
    Matrix(JocReconstructionError),
    Oamd(OamdParseError),
}

impl fmt::Display for JocRenderError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Invalid(context) => write!(formatter, "invalid {context}"),
            Self::Unsupported(context) => write!(formatter, "unsupported {context}"),
            Self::Matrix(error) => write!(formatter, "JOC matrix: {error}"),
            Self::Oamd(error) => write!(formatter, "OAMD: {error}"),
        }
    }
}

impl From<JocReconstructionError> for JocRenderError {
    fn from(error: JocReconstructionError) -> Self {
        Self::Matrix(error)
    }
}

impl From<OamdParseError> for JocRenderError {
    fn from(error: OamdParseError) -> Self {
        Self::Oamd(error)
    }
}

// internal — exposed for tests/fuzz; not part of the stable API
#[doc(hidden)]
pub struct JocRenderer {
    matrices: JocMatrixDecoder,
    oamd: OamdDecoder,
    analyses: Vec<QmfAnalysis>,
    syntheses: Vec<QmfSynthesis>,
    gain_delay: Vec<[f32; 2]>,
    lfe_delay: Vec<f32>,
    delay_position: usize,
    shape: Option<(usize, usize)>,
    last_sequence: Option<u16>,
}

impl Default for JocRenderer {
    fn default() -> Self {
        Self {
            matrices: JocMatrixDecoder::default(),
            oamd: OamdDecoder::default(),
            analyses: Vec::new(),
            syntheses: Vec::new(),
            gain_delay: Vec::new(),
            lfe_delay: vec![0.0; QMF_RECONSTRUCTION_DELAY],
            delay_position: 0,
            shape: None,
            last_sequence: None,
        }
    }
}

impl JocRenderer {
    #[doc(hidden)]
    pub fn reset(&mut self) {
        self.matrices.reset();
        self.oamd.reset();
        self.analyses.clear();
        self.syntheses.clear();
        self.gain_delay.clear();
        self.lfe_delay.fill(0.0);
        self.delay_position = 0;
        self.shape = None;
        self.last_sequence = None;
    }

    /// Render one 1,536-sample, bitstream-order E-AC-3 frame to interleaved stereo.
    /// For the currently standardized five-channel JOC configurations, input
    /// order is `L, R, C, (LFE), Ls, Rs`.
    #[allow(clippy::too_many_lines)]
    #[doc(hidden)]
    pub fn render_stereo(
        &mut self,
        metadata: &JocMetadata,
        pcm: &[f32],
        source_channels: usize,
        acmod: u8,
        lfeon: bool,
    ) -> Result<Vec<f32>, JocRenderError> {
        if metadata.num_channels != 5 || !matches!(metadata.downmix_config, 0 | 3) {
            return Err(JocRenderError::Unsupported(
                "non-5.X JOC downmix configuration",
            ));
        }
        if acmod != 7 {
            return Err(JocRenderError::Unsupported(
                "JOC base presentation other than 3/2",
            ));
        }
        let expected_channels = 5 + usize::from(lfeon);
        if source_channels != expected_channels {
            return Err(JocRenderError::Invalid("JOC source channel count"));
        }
        if pcm.len() != SAMPLES_PER_JOC_FRAME * source_channels {
            return Err(JocRenderError::Invalid("JOC PCM frame length"));
        }

        let num_objects = usize::from(metadata.num_objects);
        let num_channels = usize::from(metadata.num_channels);
        if !(1..=MAX_JOC_OBJECTS).contains(&num_objects) {
            return Err(JocRenderError::Invalid("JOC object count"));
        }

        let shape = (num_channels, num_objects);
        let expected_sequence =
            self.last_sequence
                .map(|sequence| if sequence == 1023 { 1 } else { sequence + 1 });
        if metadata.sequence_count != 0
            && (expected_sequence != Some(metadata.sequence_count) || self.shape != Some(shape))
        {
            self.reset();
            return Err(JocRenderError::Unsupported(
                "JOC sequence without an initialization frame",
            ));
        }
        if metadata.sequence_count == 0 {
            self.reset();
        }
        self.ensure_shape(shape);
        self.last_sequence = Some(metadata.sequence_count);

        let oamd = self.oamd.decode(&metadata.oamd_payload)?;
        if oamd
            .kinds
            .iter()
            .any(|kind| !matches!(kind, OamdObjectKind::Lfe | OamdObjectKind::Dynamic))
        {
            return Err(JocRenderError::Unsupported(
                "non-dynamic OAMD program assignment",
            ));
        }
        let object_indices: Vec<_> = oamd.joc_object_indices().collect();
        if object_indices.len() != num_objects
            || oamd.starting_objects.len() != oamd.kinds.len()
            || oamd
                .updates
                .iter()
                .any(|update| update.objects.len() != oamd.kinds.len())
        {
            return Err(JocRenderError::Invalid("OAMD/JOC object mapping"));
        }
        if oamd
            .updates
            .windows(2)
            .any(|updates| updates[0].timing.start_sample > updates[1].timing.start_sample)
        {
            return Err(JocRenderError::Invalid("OAMD update ordering"));
        }
        validate_oamd_timing(&oamd)?;

        let matrices = self.matrices.interpolate(metadata)?;
        if matrices.num_channels() != num_channels || matrices.num_objects() != num_objects {
            return Err(JocRenderError::Invalid("JOC matrix shape"));
        }

        let mut object_pcm = vec![0.0_f32; num_objects * SAMPLES_PER_JOC_FRAME];
        for timeslot in 0..QMF_TIMESLOTS_PER_EAC3_FRAME {
            let mut inputs = vec![[Complex32::default(); QMF_SUBBANDS]; num_channels];
            for (channel, input) in inputs.iter_mut().enumerate() {
                let source_channel = joc_to_bitstream_channel(channel);
                let mut block = [0.0_f32; QMF_SUBBANDS];
                for (sample, value) in block.iter_mut().enumerate() {
                    let frame = timeslot * QMF_SUBBANDS + sample;
                    *value = pcm[frame * source_channels + source_channel];
                }
                *input = self.analyses[channel].process(&block);
            }

            for (object, synthesis) in self.syntheses.iter_mut().enumerate() {
                let mut reconstructed = [Complex32::default(); QMF_SUBBANDS];
                for (subband, value) in reconstructed.iter_mut().enumerate() {
                    for (channel, input) in inputs.iter().enumerate() {
                        let coefficient = matrices.coefficient(object, timeslot, channel, subband);
                        value.re = input[subband].re.mul_add(coefficient, value.re);
                        value.im = input[subband].im.mul_add(coefficient, value.im);
                    }
                }
                if synthesis.is_silent()
                    && reconstructed.iter().all(|sample| {
                        sample.re.abs() <= f32::EPSILON && sample.im.abs() <= f32::EPSILON
                    })
                {
                    continue;
                }
                let output = synthesis.process(&reconstructed);
                let start = object * SAMPLES_PER_JOC_FRAME + timeslot * QMF_SUBBANDS;
                object_pcm[start..start + QMF_SUBBANDS].copy_from_slice(&output);
            }
        }

        let mut stereo = Vec::with_capacity(SAMPLES_PER_JOC_FRAME * 2);
        for sample in 0..SAMPLES_PER_JOC_FRAME {
            let delay = self.delay_position;
            let mut mixed = [0.0_f32; 2];
            for (object, &oamd_index) in object_indices.iter().enumerate() {
                let current_gain = stereo_gains_at(&oamd, oamd_index, sample);
                let delay_index = object * QMF_RECONSTRUCTION_DELAY + delay;
                let delayed_gain = self.gain_delay[delay_index];
                self.gain_delay[delay_index] = current_gain;
                let value = object_pcm[object * SAMPLES_PER_JOC_FRAME + sample];
                mixed[0] = value.mul_add(delayed_gain[0], mixed[0]);
                mixed[1] = value.mul_add(delayed_gain[1], mixed[1]);
            }

            if lfeon {
                // E-AC-3 keeps LFE after every full-bandwidth channel in
                // bitstream order, so acmod 3/2 + LFE uses slot 5.
                let current_lfe = pcm[sample * source_channels + source_channels - 1];
                let delayed_lfe = self.lfe_delay[delay];
                self.lfe_delay[delay] = current_lfe;
                mixed[0] = delayed_lfe.mul_add(LFE_STEREO_GAIN, mixed[0]);
                mixed[1] = delayed_lfe.mul_add(LFE_STEREO_GAIN, mixed[1]);
            }

            stereo.push(mixed[0]);
            stereo.push(mixed[1]);
            self.delay_position += 1;
            if self.delay_position == QMF_RECONSTRUCTION_DELAY {
                self.delay_position = 0;
            }
        }
        Ok(stereo)
    }

    fn ensure_shape(&mut self, shape: (usize, usize)) {
        if self.shape == Some(shape) {
            return;
        }
        let (num_channels, num_objects) = shape;
        self.analyses = vec![QmfAnalysis::default(); num_channels];
        self.syntheses = vec![QmfSynthesis::default(); num_objects];
        self.gain_delay = vec![[0.0; 2]; num_objects * QMF_RECONSTRUCTION_DELAY];
        self.lfe_delay.fill(0.0);
        self.delay_position = 0;
        self.shape = Some(shape);
    }
}

fn validate_oamd_timing(frame: &OamdFrame) -> Result<(), JocRenderError> {
    let mut previous_end = 0_usize;
    for update in &frame.updates {
        let start = usize::from(update.timing.start_sample);
        let end = start
            .checked_add(usize::from(update.timing.ramp_duration))
            .ok_or(JocRenderError::Invalid("OAMD ramp timing"))?;
        if start < previous_end {
            return Err(JocRenderError::Unsupported("overlapping OAMD ramps"));
        }
        if start > SAMPLES_PER_JOC_FRAME || end > SAMPLES_PER_JOC_FRAME {
            return Err(JocRenderError::Unsupported("cross-frame OAMD ramp"));
        }
        previous_end = end;
    }
    Ok(())
}

fn joc_to_bitstream_channel(channel: usize) -> usize {
    // TS 103 420 JOC order is L, R, C, Ls, Rs. The E-AC-3 acmod=7
    // decoder scratch is in bitstream order L, C, R, Ls, Rs, (LFE).
    match channel {
        0 => 0,
        1 => 2,
        2 => 1,
        3 => 3,
        4 => 4,
        _ => channel,
    }
}

fn stereo_gains_at(frame: &OamdFrame, object: usize, sample: usize) -> [f32; 2] {
    let mut from = &frame.starting_objects[object];
    let mut to = from;
    let mut alpha = 1.0_f32;
    for update in &frame.updates {
        let start = usize::from(update.timing.start_sample);
        if sample < start {
            break;
        }
        to = &update.objects[object];
        let duration = update.timing.ramp_duration;
        alpha = if duration == 0 {
            1.0
        } else {
            let elapsed = u16::try_from(sample - start)
                .unwrap_or(u16::MAX)
                .min(duration);
            f32::from(elapsed) / f32::from(duration)
        };
        if alpha < 1.0 {
            break;
        }
        from = to;
    }
    stereo_gains(from, to, alpha)
}

fn stereo_gains(from: &OamdObjectState, to: &OamdObjectState, alpha: f32) -> [f32; 2] {
    let alpha = alpha.clamp(0.0, 1.0);
    let from_gain = if from.active { from.linear_gain() } else { 0.0 };
    let to_gain = if to.active { to.linear_gain() } else { 0.0 };
    let gain = from_gain + (to_gain - from_gain) * alpha;
    if gain == 0.0 {
        return [0.0; 2];
    }

    let x = (from.position[0] + (to.position[0] - from.position[0]) * alpha).clamp(0.0, 1.0);
    let width = (from.size[0] + (to.size[0] - from.size[0]) * alpha).clamp(0.0, 1.0);
    let mut left = (x * FRAC_PI_2).cos();
    let mut right = (x * FRAC_PI_2).sin();

    // Width one approaches an equal diffuse pair; width zero remains a
    // point source. Renormalize so spreading does not change energy.
    left += (DIFFUSE_STEREO_GAIN - left) * width;
    right += (DIFFUSE_STEREO_GAIN - right) * width;
    let norm = left.hypot(right);
    if norm > 0.0 {
        left /= norm;
        right /= norm;
    }
    [left * gain, right * gain]
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // Endpoint panning has exact values.
mod tests {
    use super::*;

    fn object(x: f32, gain_db: Option<f32>, width: f32) -> OamdObjectState {
        OamdObjectState {
            active: true,
            gain_db,
            priority: 1.0,
            position: [x, 0.0, 0.0],
            size: [width, 0.0, 0.0],
            screen_reference: false,
            channel_lock: false,
            zone_constraints: 0,
            elevation_enabled: true,
            raw_position: [0; 3],
        }
    }

    #[test]
    fn point_pan_is_equal_power() {
        let left = stereo_gains(
            &object(0.0, Some(0.0), 0.0),
            &object(0.0, Some(0.0), 0.0),
            1.0,
        );
        let center = stereo_gains(
            &object(0.5, Some(0.0), 0.0),
            &object(0.5, Some(0.0), 0.0),
            1.0,
        );
        let right = stereo_gains(
            &object(1.0, Some(0.0), 0.0),
            &object(1.0, Some(0.0), 0.0),
            1.0,
        );
        assert_eq!(left, [1.0, 0.0]);
        assert!((center[0] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1.0e-6);
        assert!((center[1] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1.0e-6);
        assert!(right[0].abs() < 1.0e-6);
        assert!((right[1] - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn gain_and_position_ramp_together() {
        let from = object(0.0, None, 0.0);
        let to = object(1.0, Some(0.0), 0.0);
        let midpoint = stereo_gains(&from, &to, 0.5);
        let expected = 0.5 * std::f32::consts::FRAC_1_SQRT_2;
        assert!((midpoint[0] - expected).abs() < 1.0e-6);
        assert!((midpoint[1] - expected).abs() < 1.0e-6);
    }

    #[test]
    fn maps_joc_order_to_eac3_bitstream_order() {
        assert_eq!(
            (0..5).map(joc_to_bitstream_channel).collect::<Vec<_>>(),
            [0, 2, 1, 3, 4]
        );
    }

    #[test]
    fn accepts_frame_local_oamd_ramps_and_rejects_ambiguous_timing() {
        let mut frame = OamdFrame {
            kinds: Vec::new(),
            starting_objects: Vec::new(),
            updates: vec![super::super::oamd::OamdUpdateBlock {
                timing: super::super::oamd::OamdTiming {
                    start_sample: 0,
                    ramp_duration: 1536,
                },
                objects: Vec::new(),
            }],
        };
        assert_eq!(validate_oamd_timing(&frame), Ok(()));

        frame.updates[0].timing.start_sample = 1;
        assert!(matches!(
            validate_oamd_timing(&frame),
            Err(JocRenderError::Unsupported("cross-frame OAMD ramp"))
        ));

        frame.updates[0].timing = super::super::oamd::OamdTiming {
            start_sample: 0,
            ramp_duration: 1000,
        };
        frame.updates.push(super::super::oamd::OamdUpdateBlock {
            timing: super::super::oamd::OamdTiming {
                start_sample: 512,
                ramp_duration: 0,
            },
            objects: Vec::new(),
        });
        assert!(matches!(
            validate_oamd_timing(&frame),
            Err(JocRenderError::Unsupported("overlapping OAMD ramps"))
        ));
    }
}
