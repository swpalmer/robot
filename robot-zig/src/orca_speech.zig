//! By convention, root.zig is the root source file when making a library. If

const std = @import("std");
const testing = std.testing;

const alsa = @cImport({
    @cInclude("alsa/asoundlib.h");
    //@cInclude("alsa/pcm.h");
});

pub fn init() !void {
    return init_orca();
}
pub fn deinit() !void {
    return deinit_orca();
}

pub fn speak(text: [:0]const u8) !void {
    return speak_orca(text);
}

// Text to speech using PICOVOICE Orca Streaming Text-to-Speech Engine
// https://picovoice.ai/docs/api/orca-c/
const pv = @cImport({
    @cInclude("pv_orca.h");
});

var orca: ?*pv.pv_orca_t = null;
var synthesize_params: ?*pv.pv_orca_synthesize_params_t = null;
var orca_sample_rate: i32 = 16000; // replaced by API call
var max_characters: i32 = 256; // replaced by API call

fn init_orca() !void {
    // Read ACCESS_KEY from a file
    const ACCESS_KEY_PATH = "picovoice_api_token.txt";

    var file = try std.fs.cwd().openFile(ACCESS_KEY_PATH, .{});
    defer file.close();
    var buf: [128]u8 = undefined;
    const n = try file.readAll(&buf);
    const access_key_slice = std.mem.trim(u8, buf[0..n], " \r\n");

    // Copy to a null-terminated buffer
    var access_key_c: [129]u8 = undefined;
    if (access_key_slice.len >= access_key_c.len)
        return error.AccessKeyTooLong;
    std.mem.copyForwards(u8, access_key_c[0..access_key_slice.len], access_key_slice);
    access_key_c[access_key_slice.len] = 0; // null terminator

    //const model_path = "orca_params_en_male.pv";
    const model_path = "orca_params_en_female.pv";

    var status: pv.pv_status_t = pv.pv_orca_init(access_key_c[0..access_key_slice.len :0].ptr, // C string pointer
        model_path, &orca);

    if (status != pv.PV_STATUS_SUCCESS) {
        std.debug.print("Orca init failed with status: {}\n", .{status});
        // error handling logic
        return error.OrcaInitFailed;
    }

    status = pv.pv_orca_sample_rate(orca, &orca_sample_rate);
    std.debug.print("Orca sample rate: {}\n", .{orca_sample_rate});
    status = pv.pv_orca_max_character_limit(orca, &max_characters);
    std.debug.print("Orca max characters: {}\n", .{max_characters});

    status = pv.pv_orca_synthesize_params_init(&synthesize_params);
    if (status != pv.PV_STATUS_SUCCESS) {
        std.debug.print("Orca synthesize params init failed with status: {}\n", .{status});
        // error handling logic
        return error.OrcaSynthesizeParamsInitFailed;
    }

    // change the default parameters of synthesize_params as desired
    status = pv.pv_orca_synthesize_params_set_speech_rate(synthesize_params, 1.0); // 0.7 to 1.3
    if (status != pv.PV_STATUS_SUCCESS) {
        std.debug.print("Orca synthesize params set_speech_rate failed with status: {}\n", .{status});
        // error handling logic
        return error.OrcaSynthesizeParamsSetSpeechRateFailed;
    }

    status = pv.pv_orca_synthesize_params_set_random_state(synthesize_params, 0);
    if (status != pv.PV_STATUS_SUCCESS) {
        std.debug.print("Orca synthesize params set_random_state failed with status: {}\n", .{status});
        // error handling logic
        return error.OrcaSynthesizeParamsSetRandomStateFailed;
    }
}

fn deinit_orca() void {
    pv.pv_orca_synthesize_params_delete(synthesize_params);
    pv.pv_orca_delete(orca);
}

fn speak_orca(text: [:0]const u8) !void {
    if (text.len > max_characters) {
        std.debug.print("Orca text too long: {} > {}\n", .{ text.len, max_characters });
        return error.OrcaTextTooLong;
    }

    var num_samples: i32 = 0;
    var synthesized_pcm: ?*i16 = null;
    var num_alignments: i32 = 0;
    var alignments: [*c][*c]pv.pv_orca_word_alignment_t = null;
    const status = pv.pv_orca_synthesize(orca, text, synthesize_params, &num_samples, &synthesized_pcm, &num_alignments, &alignments);
    if (status != pv.PV_STATUS_SUCCESS) {
        // error handling logic
        std.debug.print("Orca synthesize failed with status: {}\n", .{status});
        return error.OrcaSynthesizeFailed;
    }
    defer pv.pv_orca_pcm_delete(synthesized_pcm);
    defer _ = pv.pv_orca_word_alignments_delete(num_alignments, alignments);
    // I think we need to send the samples to the audio device

    std.debug.print("Orca synthesized {} samples\n", .{num_samples});
    std.debug.print("Orca synthesized {} alignments\n", .{num_alignments});

    const nonnull_ptr = synthesized_pcm orelse return error.NullPointer;
    const sample_slice = @as([*]i16, @ptrCast(nonnull_ptr))[0..@intCast(num_samples)];

    std.debug.print("Orca sample rate is {} and slice length is {}\n", .{ orca_sample_rate, sample_slice.len });
    std.debug.print("Num samples is {}\n", .{num_samples});
    //try playMonoSamples(sample_slice, @as(u32, @intCast(orca_sample_rate)));
    //try playAudioBuffer(sample_slice, orca_sample_rate, 1);
    //try playMonoSamples(sample_slice, 48000);
    try oldPlayback(synthesized_pcm, num_samples);
}

fn oldPlayback(synthesized_pcm: ?*i16, num_samples: i32) !void {
    // TODO: make this a function to send samples to audio device
    var playback_handle: ?*alsa.snd_pcm_t = null;
    var err = alsa.snd_pcm_open(&playback_handle, "plughw:0,0", alsa.SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
        std.debug.print("Error opening alsa playback handle: {}\n", .{err});
        return error.AlsaPlaybackHandleOpenFailed;
    }
    std.debug.print("Opened alsa playback handle.\n", .{});
    defer _ = alsa.snd_pcm_close(playback_handle);

    // var hw_params: ?*alsa.snd_pcm_hw_params_t = null;

    // err = alsa.snd_pcm_hw_params_malloc(&hw_params);
    // if (err < 0) {
    //     std.debug.print("Error allocating alsa hw params: {}\n", .{err});
    //     return error.AlsaHwParamsAllocFailed;
    // }
    // std.debug.print("Allocated alsa HW Params.\n", .{});
    // defer alsa.snd_pcm_hw_params_free(hw_params);

    // err = alsa.snd_pcm_hw_params_any(playback_handle, hw_params);
    // if (err < 0) {
    //     std.debug.print("Error calling snd_pcm_hw_params_any: {}\n", .{err});
    //     return error.AlsaHwParamsSetFailed;
    // }
    // std.debug.print("Got ANY alsa HW Params:\n", .{});

    // err = alsa.snd_pcm_hw_params_set_access(playback_handle, hw_params, alsa.SND_PCM_ACCESS_RW_INTERLEAVED);
    // if (err < 0) {
    //     std.debug.print("Error setting snd_pcm_hw_params_set_access: {}\n", .{err});
    //     return error.AlsaHwParamsSetFailed;
    // }
    // std.debug.print("Set alsa HW Params access.\n", .{});

    // err = alsa.snd_pcm_hw_params_set_format(playback_handle, hw_params, alsa.SND_PCM_FORMAT_S16_LE);
    // if (err < 0) {
    //     std.debug.print("Error setting snd_pcm_hw_params_set_format: {}\n", .{err});
    //     return error.AlsaHwParamsSetFailed;
    // }
    // std.debug.print("Set alsa HW Params format.\n", .{});

    // err = alsa.snd_pcm_hw_params_set_rate_near(playback_handle, hw_params, 44100, 0);
    // if (err < 0) {
    //     std.debug.print("Error setting snd_pcm_hw_params_set_rate_near: {}\n", .{err});
    //     return error.AlsaHwParamsSetFailed;
    // }
    // std.debug.print("Set alsa HW Params rate.\n", .{});

    // err = alsa.snd_pcm_hw_params_set_channels(playback_handle, hw_params, 1);
    // if (err < 0) {
    //     std.debug.print("Error setting snd_pcm_hw_params_set_channels: {}\n", .{err});
    //     return error.AlsaHwParamsSetFailed;
    // }
    // std.debug.print("Set alsa HW Params channels.\n", .{});

    // err = alsa.snd_pcm_hw_params(playback_handle, hw_params);
    // if (err < 0) {
    //     std.debug.print("Error setting alsa hw params: {}\n", .{err});
    //     return error.AlsaHwParamsSetFailed;
    // }
    // std.debug.print("Configured alsa HW Params\n", .{});
    // // could free hw_params now

    // err = alsa.snd_pcm_prepare(playback_handle);
    // if (err < 0) {
    //     std.debug.print("Error preparing alsa playback handle: {}\n", .{err});
    //     return error.AlsaPlaybackHandlePrepareFailed;
    // }
    // std.debug.print("Prepared alsa playback handle\n", .{});

    err = alsa.snd_pcm_set_params(playback_handle, alsa.SND_PCM_FORMAT_S16_LE, alsa.SND_PCM_ACCESS_RW_INTERLEAVED, 1, @intCast(orca_sample_rate), 1, 500000);

    if (err < 0) {
        std.debug.print("Error setting alsa playback params: {}\n", .{err});
    }

    var samples_written: usize = 0;
    const pcm_ptr = synthesized_pcm orelse {
        std.debug.print("synthesized_pcm is null!\n", .{});
        return error.OrcaSynthesizeReturnedNull;
    };
    // Cast the C pointer to a Zig pointer for arithmetic
    const zig_pcm_ptr: [*]i16 = @ptrCast(pcm_ptr);

    while (samples_written < @as(usize, @intCast(num_samples))) {
        const to_write: c_ulong = @intCast(num_samples - @as(i32, @intCast(samples_written)));
        std.debug.print("Writing {} samples this call...\n", .{to_write});
        const ret = alsa.snd_pcm_writei(
            playback_handle,
            zig_pcm_ptr + @as(usize, @intCast(samples_written)),
            to_write,
        );
        if (ret < 0) {
            std.debug.print("Error writing to alsa playback handle: {}\n", .{ret});
            return error.AlsaPlaybackHandleWriteFailed;
        }
        samples_written += @intCast(ret);
        std.debug.print("Total written: {}\n", .{samples_written});
    }

    std.debug.print("Wrote {} samples to alsa playback handle\n", .{num_samples});
    err = alsa.snd_pcm_drain(playback_handle);
    if (err < 0) {
        std.debug.print("Error draining alsa playback handle: {}\n", .{err});
        return error.AlsaPlaybackHandleDrainFailed;
    }

    std.debug.print("Drained alsa playback handle\n", .{});
}

// Possibly move from here to a separate file later (sound.zig)

// samples should be little endian 16-bit signed integers (i16).
// sample_rate is typically 44100 Hz, but can vary based on the audio device and settings.
// channels = 1 for Mono audio, 2 for stereo
pub fn playAudioBuffer(samples: []i16, sample_rate: i32, channels: i32) !void {
    if (samples.len == 0) {
        std.debug.print("No samples to play\n", .{});
        return error.EmptyBuffer;
    }

    var pcm: ?*alsa.snd_pcm_t = null;

    // Open default playback device
    //const device = "default";
    const device = "plughw:0,0"; // for 3.5mm headphones
    if (alsa.snd_pcm_open(&pcm, device, alsa.SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        return error.OpenFailed;
    }

    // Set hardware parameters
    const format = alsa.SND_PCM_FORMAT_S16_LE;

    if (alsa.snd_pcm_set_params(
        pcm, // Pointer to the sample buffer
        format,
        alsa.SND_PCM_ACCESS_RW_INTERLEAVED,
        @as(c_uint, @intCast(channels)),
        @as(c_uint, @intCast(sample_rate)),
        1, // allow low latency
        500000, // 0.5 sec latency
    ) < 0) {
        return error.SetParamsFailed;
    }

    _ = alsa.snd_pcm_prepare(pcm); // ensure it's ready even if params succeeded

    // Play the buffer
    var total_written: usize = 0;
    const total_frames = samples.len / @as(usize, @intCast(channels));

    while (total_written < total_frames) {
        const frame_ptr = samples.ptr + (total_written * @as(usize, @intCast(channels)));
        const frames_remaining = total_frames - total_written;

        var result = alsa.snd_pcm_writei(pcm, frame_ptr, frames_remaining);
        if (result < 0) {
            std.debug.print("writei failed: {}, trying recover...\n", .{result});
            result = alsa.snd_pcm_recover(pcm, @as(c_int, @intCast(result)), 1);

            if (result < 0) {
                std.debug.print("Error: failed to recover write stream: {}\n", .{result});
                // attempt manual reset
                if (alsa.snd_pcm_prepare(pcm) < 0) {
                    return error.PcmPrepareFailed;
                }
                continue; // retry after prepare
            }
            continue; // retry after recover
        }

        total_written += @as(usize, @intCast(result));
    }

    const drain_result = alsa.snd_pcm_drain(pcm);
    if (drain_result < 0) {
        std.debug.print("Error draining PCM device: {}\n", .{drain_result});
        return error.DrainFailed;
    }
    const close_result = alsa.snd_pcm_close(pcm);
    if (close_result < 0) {
        std.debug.print("Error closing PCM device: {}\n", .{close_result});
        return error.CloseFailed;
    }
    std.debug.print("Audio playback completed successfully.\n", .{});
}

pub fn playToneMono(frequency: f32, duration: f32) !void {
    const rate = 41000;
    const allocator = std.heap.page_allocator;
    // Your sample buffer — for example, a 440Hz sine wave
    const sample_count = @round(rate * duration);
    var buffer = try allocator.alloc(i16, sample_count);
    defer allocator.free(buffer);

    var i: usize = 0;
    while (i < sample_count) : (i += 1) {
        const t = @as(f64, @floatFromInt(i)) / @as(f64, @floatFromInt(rate));
        buffer[i] = @intFromFloat(std.math.sin(2 * std.math.pi * frequency * t) * 32767.0);
    }
    playAudioBuffer(buffer, rate, 1);
}

pub fn playMonoSine44100Hz() !void {
    const allocator = std.heap.page_allocator;
    //const device = "hw:0,0";
    const device = "plughw:0,0";

    var pcm: ?*alsa.snd_pcm_t = null;
    if (alsa.snd_pcm_open(&pcm, device, alsa.SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        return error.OpenFailed;
    }

    const rate: u32 = 44100;
    const channels: u32 = 1;
    const format = alsa.SND_PCM_FORMAT_S16_LE;

    if (alsa.snd_pcm_set_params(
        pcm,
        format,
        alsa.SND_PCM_ACCESS_RW_INTERLEAVED,
        @as(c_uint, channels),
        @as(c_uint, rate),
        1,
        500_000,
    ) < 0) {
        return error.SetParamsFailed;
    }

    _ = alsa.snd_pcm_prepare(pcm); // force state to PREPARED

    const seconds: usize = 1;
    const frame_count = rate * seconds;
    const sample_count = frame_count * channels;

    var buffer = try allocator.alloc(i16, sample_count);
    defer allocator.free(buffer);

    var i: usize = 0;
    while (i < frame_count) : (i += 1) {
        const t = @as(f64, @floatFromInt(i)) / 44100.0;
        const val = std.math.sin(2.0 * std.math.pi * 440.0 * t);
        buffer[i] = @as(i16, @intFromFloat(val * 32767.0));
    }

    var total_written: usize = 0;
    while (total_written < frame_count) {
        const ptr = buffer.ptr + total_written;
        const remaining = frame_count - total_written;

        var result = alsa.snd_pcm_writei(pcm, ptr, remaining);
        if (result < 0) {
            result = alsa.snd_pcm_recover(pcm, @as(c_int, @intCast(result)), 1);
            if (result < 0) {
                std.debug.print("Write failed: {}\n", .{result});
                return error.WriteFailed;
            }
            continue;
        }

        total_written += @as(usize, @intCast(result));
    }

    _ = alsa.snd_pcm_drain(pcm);
    _ = alsa.snd_pcm_close(pcm);

    std.debug.print("Playback finished.\n", .{});
}

pub fn playMonoSamples(samples: []i16, sample_rate: u32) !void {
    const device = "hw:0,0"; // or "plughw:0,0" for resampling
    const format = alsa.SND_PCM_FORMAT_S16_LE;
    const channels: u32 = 1;

    var pcm: ?*alsa.snd_pcm_t = null;

    const open_result = alsa.snd_pcm_open(&pcm, device, alsa.SND_PCM_STREAM_PLAYBACK, 0);
    if (open_result < 0) return error.OpenFailed;

    // Use manual param setting instead of snd_pcm_set_params
    const access = alsa.SND_PCM_ACCESS_RW_INTERLEAVED;

    if (alsa.snd_pcm_set_params(
        pcm,
        format,
        access,
        @as(c_uint, channels),
        @as(c_uint, sample_rate),
        1, // allow ALSA to resample
        500000, // latency in us (0.5 sec)
    ) < 0) {
        _ = alsa.snd_pcm_close(pcm);
        return error.SetParamsFailed;
    }

    _ = alsa.snd_pcm_prepare(pcm);

    const frame_count = samples.len / @as(usize, channels);

    var total_written: usize = 0;

    while (total_written < frame_count) {
        const remaining = frame_count - total_written;
        const ptr = samples.ptr + total_written * channels;

        const result = alsa.snd_pcm_writei(pcm, ptr, remaining);
        if (result < 0) {
            const recovered = alsa.snd_pcm_recover(pcm, @as(c_int, @intCast(result)), 1);
            if (recovered < 0) {
                const err_str = alsa.snd_strerror(@as(c_int, @intCast(recovered)));
                std.debug.print("ALSA write failed permanently: {} ({s})\n", .{ recovered, err_str });
                _ = alsa.snd_pcm_close(pcm);
                return error.WriteFailed;
            }
            continue;
        }

        total_written += @as(usize, @intCast(result));
    }

    _ = alsa.snd_pcm_drain(pcm);
    _ = alsa.snd_pcm_close(pcm);

    std.debug.print("Playback complete.\n", .{});
}
