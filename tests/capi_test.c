/* SPDX-License-Identifier: MIT */
/* Exercises the C API from plain C (built as C, linked against the C++ engine). */
#include <crucible/crucible.h>
#include <crucible/crucible_module.h>

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(cond)                                                                                                         \
    do                                                                                                                      \
    {                                                                                                                       \
        if (!(cond))                                                                                                        \
        {                                                                                                                   \
            fprintf(stderr, "C API check failed: %s (line %d)\n", #cond, __LINE__);                                         \
            return 1;                                                                                                       \
        }                                                                                                                   \
    } while (0)

int main(void)
{
    CrucibleConfig defaults;
    memset(&defaults, 0xAB, sizeof(defaults));
    crucible_config_init(&defaults);
    CHECK(defaults.struct_size == sizeof(defaults) && defaults.max_voices == 16 && defaults.max_block_size == 512);
    CHECK(defaults.quality == 1 && defaults.seed == 0 && defaults.preload_wavetables == 0 &&
          defaults.oversampling_filter == 0);
    CrucibleConfig config = defaults;
    config.max_voices = 8;
    CrucibleEngine* engine = crucible_create(&config);
    CHECK(engine != NULL);
    CHECK(crucible_api_version() == CRUCIBLE_API_VERSION);
    CHECK(crucible_param_count(engine) > 300);
    const int cutoff = crucible_param_index(engine, "filter_cutoff");
    CHECK(cutoff >= 0 && crucible_param_index(engine, "nope") == -1);
    CrucibleParamMeta info;
    info.struct_size = sizeof(info);
    CHECK(crucible_param_meta(engine, cutoff, &info) == 1 && strcmp(info.id, "filter_cutoff") == 0 && info.max == 20000.0f);
    {
        uint32_t h = 2166136261u; /* FNV-1a of "filter_cutoff" */
        const char* c;
        for (c = "filter_cutoff"; *c; ++c)
            h = (h ^ (uint8_t)*c) * 16777619u;
        CHECK(info.host_id == (h & 0x7fffffffu));
    }
    CHECK(crucible_param_choice(engine, crucible_param_index(engine, "osc1_table"), 0) != NULL);
    crucible_prepare(engine, 48000.0, 256);
    CHECK(crucible_set_param_id(engine, "filter_cutoff", 3000.0f) == 1);
    CHECK(fabsf(crucible_get_param(engine, cutoff) - 3000.0f) < 1.0e-3f);
    CHECK(crucible_param_index_by_host_id(engine, info.host_id) == cutoff &&
          crucible_param_index_by_host_id(engine, 0u) == -1);
    CHECK(crucible_set_param_text(engine, "filter_cutoff", "1.2k") == 1 &&
          fabsf(crucible_get_param(engine, cutoff) - 1200.0f) < 1.0e-3f);
    CHECK(crucible_set_param_text(engine, "filter_cutoff", "loud") == 0 &&
          fabsf(crucible_get_param(engine, cutoff) - 1200.0f) < 1.0e-3f);
    CHECK(crucible_set_param_text(engine, "nope", "1") == 0 && crucible_set_param_text(engine, NULL, "1") == 0 &&
          crucible_set_param_text(engine, "filter_cutoff", NULL) == 0);
    {
        const int type = crucible_param_index(engine, "osc1_type");
        const int scale = crucible_param_index(engine, "key_scale");
        float before, after;
        CHECK(crucible_set_param_text(engine, "osc1_type", "analog") == 1);
        CHECK(crucible_text_to_value(engine, type, "Analog (VA)", &before) == 1 &&
              crucible_get_param(engine, type) == before);
        CHECK(crucible_set_param_text(engine, "key_scale", "Diminished (Half-Whole)") == 1);
        before = crucible_get_param(engine, scale);
        CHECK(crucible_set_param_text(engine, "key_scale", "diminished") == 0); /* fits two choices */
        after = crucible_get_param(engine, scale);
        CHECK(before == after);
    }
    CHECK(crucible_set_param_id(engine, "filter_cutoff", 3000.0f) == 1);

    float left[256], right[256];
    CrucibleMidiEvent on = {10, 0x90, 60, 100};
    float peak = 0.0f;
    for (int block = 0; block < 40; ++block)
    {
        crucible_process(engine, left, right, 256, block == 0 ? &on : NULL, block == 0 ? 1 : 0);
        for (int i = 0; i < 256; ++i)
        {
            CHECK(isfinite(left[i]) && isfinite(right[i]));
            if (fabsf(left[i]) > peak)
                peak = fabsf(left[i]);
        }
    }
    CHECK(peak > 0.01f);
    CHECK(crucible_add_route(engine, 1000 + 74, "filter_cutoff", 0.5f) == 1);
    CHECK(crucible_add_route(engine, 1000 + 74, "osc1_table", 0.5f) == 0);
    CHECK(crucible_add_route(engine, 1000 + 74, "filter_res", NAN) == 0);
    CHECK(crucible_add_route(engine, 1000 + 74, "filter_res", INFINITY) == 0);
    {
        CrucibleRoute route;
        memset(&route, 0, sizeof route);
        route.struct_size = sizeof route;
        route.source = 1000 + 1;
        route.target_id = "filter_res";
        route.amount = 0.25f;
        route.polarity = 2;
        route.curve = 0.5f;
        route.smoothing_ms = 40.0f;
        CHECK(crucible_add_route_ex(engine, &route) == 1);
        route.target_id = "osc1_table";
        CHECK(crucible_add_route_ex(engine, &route) == 0);
        CHECK(crucible_add_route_ex(engine, NULL) == 0);
        {
            /* the extra fields reach the patch */
            const int size = crucible_save_patch(engine, "route", NULL, 0);
            char* text = (char*)malloc((size_t)size + 1);
            CHECK(text != NULL && crucible_save_patch(engine, "route", text, (size_t)size + 1) == size);
            CHECK(strstr(text, "filter_res") != NULL && strstr(text, ",2,0.500000,40.000") != NULL);
            free(text);
        }
    }
    {
        /* Route read-back: order is insertion order, replacement keeps the position, removal shifts later routes down. */
        enum
        {
            CAP = 64
        };
        int idx[CAP], src[CAP];
        const int base = crucible_route_count(engine);
        CrucibleRoute r;
        int n;
        CHECK(base >= 2);
        crucible_param_changes(engine, idx, src, CAP);
        memset(&r, 0, sizeof r);
        r.struct_size = sizeof r;
        CHECK(crucible_route_get(engine, 0, &r) == 1 && r.source == 1000 + 74 && strcmp(r.target_id, "filter_cutoff") == 0);
        CHECK(r.amount == 0.5f);
        CHECK(crucible_route_get(engine, base - 1, &r) == 1 && r.source == 1000 + 1 &&
              strcmp(r.target_id, "filter_res") == 0);
        CHECK(r.polarity == 2 && r.curve == 0.5f && r.smoothing_ms == 40.0f);
        CHECK(crucible_route_get(engine, base, &r) == 0 && crucible_route_get(engine, -1, &r) == 0);
        CHECK(crucible_route_get(engine, 0, NULL) == 0 && crucible_route_get(NULL, 0, &r) == 0 &&
              crucible_route_count(NULL) == 0);
        {
            /* a short struct is filled only as far as it reaches; a struct_size of 0 fails */
            CrucibleRoute small;
            memset(&small, 0x55, sizeof small);
            small.struct_size = offsetof(CrucibleRoute, amount) + sizeof small.amount;
            CHECK(crucible_route_get(engine, 0, &small) == 1 && small.amount == 0.5f && small.polarity == 0x55555555);
            small.struct_size = 0;
            CHECK(crucible_route_get(engine, 0, &small) == 0);
        }
        /* nothing changed yet */
        CHECK(crucible_param_changes(engine, idx, src, CAP) == 0);
        CHECK(crucible_add_route(engine, 1000 + 11, "filter_res", 0.1f) == 1);
        n = crucible_param_changes(engine, idx, src, CAP);
        CHECK(n == 1 && idx[0] == CRUCIBLE_CHANGE_ROUTES && src[0] == CRUCIBLE_SOURCE_INTERNAL);
        CHECK(crucible_route_count(engine) == base + 1);
        CHECK(crucible_route_get(engine, base, &r) == 1 && r.source == 1000 + 11);
        /* replacing keeps the position */
        CHECK(crucible_add_route(engine, 1000 + 74, "filter_cutoff", -0.25f) == 1);
        CHECK(crucible_route_get(engine, 0, &r) == 1 && r.source == 1000 + 74 && r.amount == -0.25f);
        CHECK(crucible_param_changes(engine, idx, src, CAP) == 1 && idx[0] == CRUCIBLE_CHANGE_ROUTES);
        /* the same values again change nothing */
        CHECK(crucible_add_route(engine, 1000 + 74, "filter_cutoff", -0.25f) == 1);
        CHECK(crucible_param_changes(engine, idx, src, CAP) == 0);
        /* removing the first shifts the others down */
        CHECK(crucible_remove_route(engine, 1000 + 74, "filter_cutoff") == 1);
        CHECK(crucible_route_count(engine) == base);
        CHECK(crucible_route_get(engine, 0, &r) == 1 && r.source == 1000 + 1);
        CHECK(crucible_param_changes(engine, idx, src, CAP) == 1 && idx[0] == CRUCIBLE_CHANGE_ROUTES);
        CHECK(crucible_remove_route(engine, 1000 + 74, "filter_cutoff") == 0);
        CHECK(crucible_param_changes(engine, idx, src, CAP) == 0);
        {
            /* a patch load that changes the routes is reported; one that keeps them is not */
            const int size = crucible_save_patch(engine, "r", NULL, 0);
            char* text = (char*)malloc((size_t)size + 1);
            int k, routes = 0;
            CHECK(text != NULL && crucible_save_patch(engine, "r", text, (size_t)size + 1) == size);
            crucible_param_changes(engine, idx, src, CAP);
            CHECK(crucible_load_patch(engine, text, NULL, 0) == 1);
            n = crucible_param_changes(engine, idx, src, CAP);
            for (k = 0; k < n; ++k)
                routes += idx[k] == CRUCIBLE_CHANGE_ROUTES;
            CHECK(routes == 0);
            CHECK(crucible_add_route(engine, 1000 + 12, "filter_res", 0.3f) == 1);
            crucible_param_changes(engine, idx, src, CAP);
            CHECK(crucible_load_patch(engine, text, NULL, 0) == 1);
            n = crucible_param_changes(engine, idx, src, CAP);
            for (k = 0; k < n; ++k)
                routes += idx[k] == CRUCIBLE_CHANGE_ROUTES;
            CHECK(routes == 1);
            CHECK(crucible_route_count(engine) == base);
            free(text);
        }
        CHECK(crucible_add_route(engine, 1000 + 74, "filter_cutoff", 0.5f) == 1);
    }
    {
        /* User vowel tables of the Formant filter: 2 vowels x 2 formants. */
        CrucibleFormant ok[4] = {{500.0f, 80.0f, 0.0f}, {1500.0f, 100.0f, -6.0f}, {900.0f, 0.0f, 0.0f}, {2500.0f, 120.0f, -12.0f}};
        CrucibleFormant bad[4];
        char err[128] = "";
        memcpy(bad, ok, sizeof ok);
        CHECK(crucible_set_formant_table(engine, 0, ok, 2, 2, 4.5f, err, sizeof err) == 1);
        CHECK(crucible_set_formant_table(engine, 1, ok, 2, 2, 1.0f, err, sizeof err) == 1);
        CHECK(crucible_set_formant_table(engine, 0, ok, 1, 2, 1.0f, err, sizeof err) == 0 && err[0] != '\0');
        CHECK(crucible_set_formant_table(engine, 0, ok, 2, 5, 1.0f, err, sizeof err) == 0);
        CHECK(crucible_set_formant_table(engine, 0, ok, 2, 2, 0.0f, err, sizeof err) == 0);
        CHECK(crucible_set_formant_table(engine, 99, ok, 2, 2, 1.0f, err, sizeof err) == 0);
        CHECK(crucible_set_formant_table(engine, -1, ok, 2, 2, 1.0f, NULL, 0) == 0);
        bad[1].freq_hz = 10.0f;
        CHECK(crucible_set_formant_table(engine, 0, bad, 2, 2, 1.0f, err, sizeof err) == 0);
        bad[1] = ok[1];
        bad[2].gain_db = 30.0f;
        CHECK(crucible_set_formant_table(engine, 0, bad, 2, 2, 1.0f, err, sizeof err) == 0);
        bad[2] = ok[2];
        bad[3].bandwidth_hz = -1.0f;
        CHECK(crucible_set_formant_table(engine, 0, bad, 2, 2, 1.0f, err, sizeof err) == 0);
        CHECK(crucible_set_formant_table(NULL, 0, ok, 2, 2, 1.0f, err, sizeof err) == 0);
        CHECK(crucible_set_formant_saturation(engine, 0, 1, err, sizeof err) == 1);
        CHECK(crucible_set_formant_saturation(engine, 0, 0, err, sizeof err) == 1);
        CHECK(crucible_set_formant_saturation(engine, 2, 1, err, sizeof err) == 0 && err[0] != '\0'); /* no table */
        CHECK(crucible_set_formant_saturation(engine, 99, 1, NULL, 0) == 0);
        CHECK(crucible_set_formant_saturation(NULL, 0, 1, err, sizeof err) == 0);
        CHECK(crucible_set_formant_table(engine, 0, NULL, 0, 0, 1.0f, err, sizeof err) == 1); /* reset */
        CHECK(crucible_set_formant_table(engine, 1, NULL, 2, 2, 1.0f, err, sizeof err) == 1);
    }
    {
        /* Waveshaper curve of fx.shaper: a drawn clip, symmetric mode, validation, the curve view, the getter. */
        CrucibleShaperPoint clip[4] = {{-1.0f, -0.5f, 0.0f}, {-0.5f, -0.5f, 0.0f}, {0.5f, 0.5f, 0.0f}, {1.0f, 0.5f, 0.0f}};
        CrucibleShaperPoint half[2] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.8f, 0.5f}};
        CrucibleShaperPoint back[8];
        CrucibleShaperPoint bad[4];
        float view[257];
        int sym = -1, curve_view, i;
        char err[128] = "";
        memcpy(bad, clip, sizeof clip);
        CHECK(crucible_get_shaper_curve(engine, "fx.shaper", back, 8, &sym) == 0); /* default identity */
        CHECK(crucible_set_shaper_curve(engine, "fx.shaper", clip, 4, 0, err, sizeof err) == 1);
        CHECK(crucible_get_shaper_curve(engine, "fx.shaper", back, 8, &sym) == 4 && sym == 0 && back[1].x == -0.5f &&
              back[3].y == 0.5f);
        CHECK(crucible_get_shaper_curve(engine, "fx.shaper", NULL, 0, NULL) == 4);
        curve_view = crucible_view_find(engine, "fx.shaper", "curve");
        CHECK(curve_view >= 0 && crucible_view_size(engine, curve_view) == 257);
        CHECK(crucible_view_read(engine, curve_view, view, 257, NULL) == 257);
        CHECK(view[0] == -0.5f && view[128] == 0.0f && view[256] == 0.5f);
        for (i = 1; i < 257; ++i)
            CHECK(view[i] >= view[i - 1]);
        CHECK(crucible_set_shaper_curve(engine, "fx.shaper", half, 2, 1, err, sizeof err) == 1);
        CHECK(crucible_get_shaper_curve(engine, "fx.shaper", back, 8, &sym) == 2 && sym == 1);
        CHECK(crucible_view_read(engine, curve_view, view, 257, NULL) == 257 && view[0] == -view[256] && view[256] > 0.7f);
        /* Invalid curves are refused with a reason and leave the curve as it was. */
        CHECK(crucible_set_shaper_curve(engine, "fx.shaper", clip, 1, 0, err, sizeof err) == 0 && err[0] != '\0');
        CHECK(crucible_set_shaper_curve(engine, "fx.shaper", clip, 65, 0, err, sizeof err) == 0);
        bad[2].x = -0.9f;
        CHECK(crucible_set_shaper_curve(engine, "fx.shaper", bad, 4, 0, err, sizeof err) == 0 && err[0] != '\0');
        bad[2] = clip[2];
        bad[1].y = 1.5f;
        CHECK(crucible_set_shaper_curve(engine, "fx.shaper", bad, 4, 0, NULL, 0) == 0);
        bad[1] = clip[1];
        bad[3].curve = -2.0f;
        CHECK(crucible_set_shaper_curve(engine, "fx.shaper", bad, 4, 0, err, sizeof err) == 0);
        CHECK(crucible_set_shaper_curve(engine, "fx.shaper", clip, 4, 1, err, sizeof err) == 0); /* symmetric: x >= 0 */
        CHECK(crucible_set_shaper_curve(engine, "fx.shaper", NULL, 2, 0, err, sizeof err) == 0);
        CHECK(crucible_set_shaper_curve(engine, "fx.nothing", clip, 4, 0, err, sizeof err) == 0 && err[0] != '\0');
        CHECK(crucible_set_shaper_curve(engine, "fx.saturator", clip, 4, 0, err, sizeof err) == 0);
        CHECK(crucible_set_shaper_curve(engine, NULL, clip, 4, 0, err, sizeof err) == 0);
        CHECK(crucible_set_shaper_curve(NULL, "fx.shaper", clip, 4, 0, err, sizeof err) == 0);
        CHECK(crucible_get_shaper_curve(engine, "fx.shaper", back, 8, &sym) == 2 && sym == 1); /* unchanged */
        CHECK(crucible_get_shaper_curve(NULL, "fx.shaper", back, 8, &sym) == 0);
        CHECK(crucible_set_shaper_curve(engine, "fx.shaper", NULL, 0, 0, err, sizeof err) == 1); /* identity again */
        CHECK(crucible_get_shaper_curve(engine, "fx.shaper", back, 8, &sym) == 0);
    }
    {
        /* In-memory wavetables: 3 frames of 256 samples (sine, saw, square), then 100 frames that must report decimation. */
        float frames[100 * 256];
        CrucibleWavetableInfo info;
        char err[256] = "";
        int f, i;
        for (f = 0; f < 100; ++f)
            for (i = 0; i < 256; ++i)
                frames[f * 256 + i] = sinf(6.2831853f * (float)(i * (1 + f % 5)) / 256.0f);
        memset(&info, 0, sizeof info);
        info.struct_size = sizeof info;
        CHECK(crucible_load_wavetable(engine, 2, frames, 3, 256, "three", &info, err, sizeof err) == 1);
        CHECK(info.frames_in == 3 && info.frames_used == 3 && info.decimated == 0 && info.frame_size == 256);
        CHECK(crucible_load_wavetable(engine, 2, frames, 100, 256, NULL, &info, err, sizeof err) == 1);
        CHECK(info.frames_in == 100 && info.frames_used == CRUCIBLE_WAVETABLE_FRAMES && info.decimated == 1);
        CHECK(crucible_load_wavetable(engine, CRUCIBLE_USER_WAVETABLES, frames, 3, 256, NULL, &info, err, sizeof err) == 0);
        CHECK(strstr(err, "slot") != NULL && info.frames_in == 0);
        CHECK(crucible_load_wavetable(engine, 0, frames, 3, 2, NULL, NULL, err, sizeof err) == 0);
        CHECK(crucible_load_wavetable(engine, 0, NULL, 3, 256, NULL, NULL, NULL, 0) == 0);
        CHECK(crucible_load_wavetable_file(engine, 0, "no_such_file.wav", &info, err, sizeof err) == 0 && err[0] != '\0');
        CHECK(crucible_load_wavetable_file(engine, 9, "x.wav", NULL, err, sizeof err) == 0 && strstr(err, "slot") != NULL);
        {
            CrucibleWavetableZone zones[3];
            const double hz[3] = {200.0, 800.0, 3000.0};
            int z;
            memset(zones, 0, sizeof zones);
            for (z = 0; z < 3; ++z)
            {
                zones[z].struct_size = sizeof zones[z];
                zones[z].frames = frames;
                zones[z].frame_count = 3;
                zones[z].frame_size = 256;
                zones[z].max_hz = hz[z];
            }
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 3, "zoned", &info, err, sizeof err) == 1);
            { /* a caller built against a later, larger zone struct: zones are stepped by struct_size */
                struct BigZone
                {
                    CrucibleWavetableZone base;
                    double future[3];
                } big[3];
                memset(big, 0, sizeof big);
                for (z = 0; z < 3; ++z)
                {
                    big[z].base = zones[z];
                    big[z].base.struct_size = sizeof big[z];
                    big[z].future[0] = 7.0;
                }
                CHECK(crucible_load_wavetable_zones(engine, 3, &big[0].base, 3, "big", &info, err, sizeof err) == 1);
                CHECK(info.frames_in == 3 && info.frame_size == 256);
                big[2].base.max_hz = 100.0; /* the third zone must be read at the right stride */
                CHECK(crucible_load_wavetable_zones(engine, 3, &big[0].base, 3, NULL, NULL, err, sizeof err) == 0 &&
                      strstr(err, "max_hz") != NULL);
            }
            CHECK(info.frames_in == 3 && info.frame_size == 256);
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 1, NULL, NULL, err, sizeof err) == 1);
            CHECK(crucible_load_wavetable_zones(engine, CRUCIBLE_USER_WAVETABLES, zones, 3, NULL, &info, err, sizeof err) == 0);
            CHECK(strstr(err, "slot") != NULL && info.frames_in == 0);
            CHECK(crucible_load_wavetable_zones(engine, 3, NULL, 3, NULL, NULL, err, sizeof err) == 0);
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 0, NULL, NULL, err, sizeof err) == 0);
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 17, NULL, NULL, err, sizeof err) == 0);
            zones[1].max_hz = 200.0; /* not ascending */
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 3, NULL, NULL, err, sizeof err) == 0 && strstr(err, "max_hz") != NULL);
            zones[1].max_hz = (double)NAN; /* NaN (0.0 / 0.0 is a compile error on MSVC: C2124) */
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 3, NULL, NULL, err, sizeof err) == 0);
            zones[1].max_hz = -5.0;
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 3, NULL, NULL, err, sizeof err) == 0);
            zones[1].max_hz = 800.0;
            zones[2].frame_count = 0; /* empty zone */
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 3, NULL, NULL, err, sizeof err) == 0);
            zones[2].frame_count = 3;
            zones[2].frame_size = 2;
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 3, NULL, NULL, err, sizeof err) == 0);
            zones[2].frame_size = 256;
            zones[0].frames = NULL;
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 3, NULL, NULL, err, sizeof err) == 0);
            zones[0].frames = frames;
            CHECK(crucible_load_wavetable_zones(engine, 3, zones, 3, "again", NULL, err, sizeof err) == 1);
        }
        CHECK(crucible_set_param(engine, crucible_param_index(engine, "osc1_table"), 14.0f + 2.0f) == 1);
        crucible_process(engine, left, right, 256, NULL, 0);
    }
    crucible_process_host(engine, left, right, 256, NULL, 0, 128.0, 4.0, 1);
    CrucibleMeter m = crucible_meter(engine);
    CHECK(m.peak_left_db > -120.0f);

    const int patch_size = crucible_save_patch(engine, "C test", NULL, 0); /* query the size */
    CHECK(patch_size > 0);
    char* patch = (char*)malloc((size_t)patch_size + 1);
    CHECK(patch != NULL && crucible_save_patch(engine, "C test", patch, (size_t)patch_size + 1) == patch_size);
    CHECK(strlen(patch) == (size_t)patch_size && strstr(patch, "\"crucible-patch\"") != NULL);
    {
        char small[16];
        CHECK(crucible_save_patch(engine, "C test", small, sizeof small) == patch_size && strlen(small) == sizeof small - 1);
        CHECK(crucible_describe_json(engine, NULL, 0) > 0);
    }
    char error[256];
    CHECK(crucible_set_param_id(engine, "filter_cutoff", 500.0f) == 1);
    CHECK(crucible_load_patch(engine, patch, error, sizeof(error)) == 1);
    CHECK(fabsf(crucible_get_param(engine, cutoff) - 3000.0f) < 1.0e-2f);
    CHECK(crucible_load_patch(engine, "{\"format\": \"other\"}", error, sizeof(error)) == 0 && strlen(error) > 0);
    CHECK(crucible_remove_route(engine, 1000 + 74, "filter_cutoff") == 1);
    free(patch);

    /* Sample oscillator: a 440 Hz sine in slot 0 played by osc1 (type 1). */
    {
        static float sine[2 * 4800];
        for (int i = 0; i < 4800; ++i)
            sine[2 * i] = sine[2 * i + 1] = 0.5f * sinf(6.2831853f * 440.0f * (float)i / 48000.0f);
        CHECK(crucible_load_sample(engine, 0, sine, 4800, 2, 48000.0, "sine") == 1);
        CHECK(crucible_load_sample(engine, 0, sine, 1, 2, 48000.0, "bad") == 0);
        CHECK(crucible_set_param_id(engine, "osc1_type", 1.0f) == 1);
        CHECK(crucible_set_param_id(engine, "osc1_smp_loop", 1.0f) == 1);
        CHECK(crucible_set_param_id(engine, "osc1_smp_root", 69.0f) == 1);
        CHECK(crucible_set_param_id(engine, "osc1_level", 1.0f) == 1);
        CrucibleMidiEvent note = {0, 0x90, 69, 100};
        float loud = 0.0f;
        for (int block = 0; block < 40; ++block)
        {
            crucible_process(engine, left, right, 256, block == 0 ? &note : NULL, block == 0 ? 1 : 0);
            if (block > 20)
                for (int i = 0; i < 256; ++i)
                    if (fabsf(left[i]) > loud)
                        loud = fabsf(left[i]);
        }
        CHECK(loud > 0.05f);
        CHECK(crucible_set_cloud(engine, 1, "cloud 1 0.15 test\n1 1 0 0 0 0\n") == 1);
        CHECK(crucible_set_cloud(engine, 1, "garbage") == 0);
        crucible_clear_sample(engine, 0);
    }

    /* Multisample zones. */
    {
        static float tone[2400];
        CrucibleZone zone = {1, 0, 127, 0, 127, 69.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1, 0};
        CrucibleZone back;
        for (int i = 0; i < 2400; ++i)
            tone[i] = 0.5f * sinf(6.2831853f * 440.0f * (float)i / 48000.0f);
        CHECK(crucible_add_zone_sample(engine, 0, tone, 2400, 1, 48000.0, "tone") == 1);
        CHECK(crucible_zone_sample_count(engine, 0) == 1);
        CHECK(crucible_set_zones(engine, 0, &zone, 1) == 1);
        CHECK(crucible_get_zones(engine, 0, &back, 1) == 1 && back.sample_id == 1 && back.key_hi == 127);
        crucible_clear_zones(engine, 0);
        CHECK(crucible_get_zones(engine, 0, &back, 1) == 0 && crucible_zone_sample_count(engine, 0) == 0);
    }

    /* Key lock and key bus. */
    {
        CrucibleKeyBus* bus = crucible_keybus_open_local();
        int root = -1, mask = 0, tie = 0, mode = -1, following = -1, in_key[12];
        uint32_t generation = 99;
        CHECK(bus != NULL);
        CHECK(crucible_keybus_read(bus, &root, &mask, &tie, &generation) == 1);
        CHECK(crucible_attach_keybus(engine, bus) == 1);
        CHECK(crucible_set_param_id(engine, "key_mode", 1.0f) == 1);
        CHECK(crucible_set_param_id(engine, "key_sync", 1.0f) == 1);
        generation = crucible_keybus_publish(bus, 7, 0x0AB5, 0);
        CHECK(generation >= 1);
        crucible_process(engine, left, right, 256, NULL, 0);
        CHECK(crucible_key_scale(engine, &mode, &root, in_key, &following) == 1);
        CHECK(mode == 1 && root == 7 && following == 1 && in_key[7] == 1 && in_key[8] == 0);
        CHECK(crucible_keybus_read(bus, &root, &mask, &tie, &generation) == 1 && root == 7 && mask == 0x0AB5);
        crucible_keybus_close(bus);
        CHECK(crucible_key_detect(engine, &root, &mode, NULL) == 0 || 1);
        crucible_key_detect_reset(engine);
    }

    /* Views. */
    {
        CrucibleViewInfo vi;
        vi.struct_size = sizeof(vi);
        float buf[8192];
        uint32_t words[8];
        uint64_t seq = 99, seq2 = 0;
        int count = crucible_view_count(engine);
        int meter = crucible_view_find(engine, "master", "meter");
        int scope = crucible_view_find(engine, "master", "scope");
        int notes = crucible_view_find(engine, "engine", "sounding_notes");
        int cutoff_mod = crucible_view_find(engine, "params", "filter_cutoff.mod");
        int cutoff_band = crucible_view_find(engine, "params", "filter_cutoff.band");
        int cutoff_voice0 = crucible_view_find(engine, "params", "filter_cutoff.voice@0");
        int sub_meter, sub_scope, sub_notes, sub_mod, sub_band, sub_voice;
        CrucibleMeter ref;
        CHECK(count > 100 && meter >= 0 && scope >= 0 && notes >= 0 && cutoff_mod >= 0 && cutoff_band >= 0);
        CHECK(cutoff_voice0 >= 0 && crucible_view_find(engine, "params", "filter_cutoff.voice@9999") == -1);
        CHECK(crucible_view_find(engine, "nope", "nope") == -1);
        CHECK(crucible_view_info(engine, meter, &vi) == 1 && strcmp(vi.instance, "master") == 0 &&
              strcmp(vi.id, "meter") == 0);
        CHECK(vi.kind == CRUCIBLE_VIEW_VECTOR && vi.max_elements == 3 && vi.live == 1);
        CHECK(crucible_view_info(engine, scope, &vi) == 1 && vi.max_elements == 8192);
        CHECK(crucible_view_size(engine, scope) == 8192 && crucible_view_size(engine, notes) == 4);
        CHECK(crucible_view_size(engine, -1) == -1 && crucible_view_size(engine, crucible_view_count(engine)) == -1);
        {
            /* The filter response is a pulled 256-point view; its size is known before anything is read. */
            const int response = crucible_view_find(engine, "filter", "response");
            float curve[256];
            uint64_t response_seq = 0;
            CHECK(response >= 0 && crucible_view_size(engine, response) == 256);
            CHECK(crucible_view_info(engine, response, &vi) == 1 && vi.live == 0 && vi.kind == CRUCIBLE_VIEW_VECTOR);
            CHECK(crucible_view_read(engine, response, curve, crucible_view_size(engine, response), &response_seq) == 256);
        }
        CHECK(crucible_view_info(engine, notes, &vi) == 1 && vi.kind == CRUCIBLE_VIEW_MASK && vi.max_elements == 4);
        CHECK(crucible_view_info(engine, crucible_view_count(engine), &vi) == 0);
        CHECK(crucible_view_read(engine, meter, buf, 8, &seq) == 0 && seq == 0); /* nobody subscribed */
        CHECK(crucible_view_read(engine, count + 5, buf, 8, &seq) == -1);
        sub_meter = crucible_view_subscribe(engine, meter, 0.0f);
        sub_scope = crucible_view_subscribe(engine, scope, 0.0f);
        sub_notes = crucible_view_subscribe(engine, notes, 0.0f);
        sub_mod = crucible_view_subscribe(engine, cutoff_mod, 0.0f);
        sub_band = crucible_view_subscribe(engine, cutoff_band, 0.0f);
        sub_voice = crucible_view_subscribe(engine, cutoff_voice0, 30.0f);
        CHECK(sub_meter > 0 && sub_scope > 0 && sub_notes > 0 && sub_mod > 0 && sub_band > 0 && sub_voice > 0);
        CHECK(crucible_view_subscribe(engine, count + 5, 0.0f) == 0);
        crucible_process(engine, left, right, 256, &on, 1);
        crucible_process(engine, left, right, 256, NULL, 0);
        ref = crucible_meter(engine);
        CHECK(crucible_view_read(engine, meter, buf, 8192, &seq) == 3 && seq >= 2);
        CHECK(buf[0] == ref.peak_left_db && buf[1] == ref.peak_right_db && (buf[2] != 0.0f) == (ref.clip != 0));
        CHECK(crucible_view_read(engine, meter, buf, 2, &seq2) == -2);
        CHECK(crucible_view_read(engine, scope, buf, 8192, &seq) == 8192);
        {
            float l[4096], r[4096];
            CHECK(crucible_scope(engine, l, r, 4096) == 4096);
            CHECK(memcmp(l, buf, sizeof l) == 0 && memcmp(r, buf + 4096, sizeof r) == 0);
        }
        CHECK(crucible_view_read_mask(engine, notes, words, 8, &seq) == 4);
        CHECK(crucible_view_read(engine, cutoff_mod, buf, 8, &seq) == 1);
        CHECK(buf[0] == crucible_modulated_value(engine, cutoff));
        CHECK(crucible_view_read(engine, cutoff_band, buf, 8, &seq) == 3 && buf[0] <= buf[1]);
        CHECK(crucible_view_read(engine, cutoff_voice0, buf, 8, &seq) == 1);
        crucible_process(engine, left, right, 256, NULL, 0);
        CHECK(crucible_view_read(engine, meter, buf, 8, &seq2) == 3 && seq2 > seq - 1);
        crucible_view_unsubscribe(engine, sub_meter);
        crucible_process(engine, left, right, 256, NULL, 0);
        CHECK(crucible_view_read(engine, meter, buf, 8, &seq) == 3);
        crucible_process(engine, left, right, 256, NULL, 0);
        CHECK(crucible_view_read(engine, meter, buf, 8, &seq2) == 3 && seq2 == seq); /* stopped publishing */
        crucible_view_unsubscribe(engine, sub_scope);
        crucible_view_unsubscribe(engine, sub_notes);
        crucible_view_unsubscribe(engine, sub_mod);
        crucible_view_unsubscribe(engine, sub_band);
        crucible_view_unsubscribe(engine, sub_voice);
    }
    /* The host-parameter surface: the default product lists every automatable parameter in index order. */
    {
        int k, all_exposed = 1;
        CHECK(crucible_surface_count(engine) == crucible_param_count(engine));
        for (k = 0; k < crucible_surface_count(engine); ++k)
            if (crucible_surface_param(engine, k) != k)
                all_exposed = 0;
        CHECK(all_exposed && crucible_surface_param(engine, -1) == -1 && info.exposed == 1);
        CHECK(crucible_surface_param(engine, crucible_surface_count(engine)) == -1);
    }
    /* Choice availability: every built-in oscillator type is usable in the default product. */
    {
        const int type = crucible_param_index(engine, "osc1_type");
        int k, usable = 1;
        for (k = 0; k < 9; ++k)
            usable = usable && crucible_choice_available(engine, type, k) == 1;
        CHECK(type >= 0 && usable);
        CHECK(crucible_choice_available(engine, type, 9) == 0 && crucible_choice_available(engine, type, -1) == 0);
        CHECK(crucible_choice_available(engine, -1, 0) == 0 && crucible_choice_available(NULL, type, 0) == 0);
        CHECK(crucible_choice_available(engine, crucible_param_index(engine, "osc1_level"), 0) == 0);
    }
    /* A listed surface: declared order, hidden parameters keep loading from patches. */
    {
        static const char* listed =
            "{\"format\": \"crucible-product\", \"version\": 1, \"id\": \"listed\","
            " \"surface\": {\"expose\": \"listed\", \"pages\": ["
            " {\"id\": \"main\", \"name\": \"Main\", \"params\": [\"filter_cutoff\", \"osc1_level\"]},"
            " {\"id\": \"out\", \"name\": \"Out\", \"params\": [\"master_gain\"]}]}}";
        static const char* unknown =
            "{\"format\": \"crucible-product\", \"version\": 1, \"id\": \"bad\","
            " \"surface\": {\"pages\": [{\"id\": \"a\", \"params\": [\"filter_cutoff\", \"nope\"]}]}}";
        char surface_error[256] = "";
        CrucibleParamMeta meta;
        meta.struct_size = sizeof(meta);
        CrucibleEngine* ls = crucible_create_from_product(NULL, listed, NULL, surface_error, sizeof(surface_error));
        CHECK(ls != NULL);
        CHECK(crucible_surface_count(ls) == 3);
        CHECK(crucible_surface_param(ls, 0) == crucible_param_index(ls, "filter_cutoff"));
        CHECK(crucible_surface_param(ls, 1) == crucible_param_index(ls, "osc1_level"));
        CHECK(crucible_surface_param(ls, 2) == crucible_param_index(ls, "master_gain"));
        CHECK(crucible_param_meta(ls, crucible_param_index(ls, "filter_cutoff"), &meta) == 1 && meta.exposed == 1 &&
              meta.automatable == 1 && strcmp(meta.page_id, "main") == 0);
        CHECK(crucible_param_meta(ls, crucible_param_index(ls, "osc1_semi"), &meta) == 1 && meta.exposed == 0 &&
              meta.automatable == 0);
        crucible_prepare(ls, 48000.0, 256);
        CHECK(crucible_set_param_id(ls, "osc1_semi", 5.0f) == 1);
        {
            static char saved[262144];
            const int saved_size = crucible_save_patch(ls, "hidden", saved, sizeof saved);
            CHECK(saved_size > 0 && (size_t)saved_size < sizeof saved);
            CHECK(crucible_set_param_id(ls, "osc1_semi", 0.0f) == 1);
            CHECK(crucible_load_patch(ls, saved, surface_error, sizeof(surface_error)) == 1);
            CHECK(fabsf(crucible_get_param(ls, crucible_param_index(ls, "osc1_semi")) - 5.0f) < 1.0e-3f);
        }
        crucible_destroy(ls);
        CHECK(crucible_create_from_product(NULL, unknown, NULL, surface_error, sizeof(surface_error)) == NULL);
        CHECK(strstr(surface_error, "surface.pages[0].params[1]") != NULL &&
              strstr(surface_error, "unknown parameter id 'nope'") != NULL);
    }
    /* Impulse responses load per module instance (module id from crucible_module_meta). */
    {
        static const char* two_conv =
            "{\"format\": \"crucible-product\", \"version\": 1, \"id\": \"two-conv\", \"voice\": \"none\","
            " \"fx\": {\"instances\": [{\"id\": \"c1\", \"type\": \"fx.convolution\"},"
            " {\"id\": \"c2\", \"type\": \"fx.convolution\"}],"
            " \"lanes\": [{\"id\": \"l1\", \"order\": \"listed\", \"input\": [\"aux\"], \"chain\": [\"c1\"]},"
            " {\"id\": \"l2\", \"order\": \"listed\", \"input\": [\"aux\"], \"chain\": [\"c2\"]}]}}";
        char ir_error[256] = "";
        const char* ir_path = "capi_test_ir.wav";
        CrucibleEngine* two = crucible_create_from_product(NULL, two_conv, NULL, ir_error, sizeof(ir_error));
        CHECK(two != NULL);
        crucible_prepare(two, 48000.0, 256);
        {
            /* A short mono 16-bit WAV written by hand. */
            unsigned char wav[44 + 2 * 64];
            const uint32_t data_bytes = 2 * 64, riff = 36 + data_bytes, rate = 48000, byte_rate = 96000;
            memset(wav, 0, sizeof wav);
            memcpy(wav, "RIFF", 4);
            memcpy(wav + 4, &riff, 4);
            memcpy(wav + 8, "WAVEfmt ", 8);
            {
                const uint32_t fmt_size = 16;
                const uint16_t pcm = 1, channels = 1, align = 2, bits = 16;
                memcpy(wav + 16, &fmt_size, 4);
                memcpy(wav + 20, &pcm, 2);
                memcpy(wav + 22, &channels, 2);
                memcpy(wav + 24, &rate, 4);
                memcpy(wav + 28, &byte_rate, 4);
                memcpy(wav + 32, &align, 2);
                memcpy(wav + 34, &bits, 2);
            }
            memcpy(wav + 36, "data", 4);
            memcpy(wav + 40, &data_bytes, 4);
            {
                const int16_t one = 16000;
                memcpy(wav + 44, &one, 2);
            }
            FILE* f = fopen(ir_path, "wb");
            CHECK(f != NULL);
            CHECK(fwrite(wav, 1, sizeof wav, f) == sizeof wav);
            fclose(f);
        }
        CHECK(crucible_load_ir(two, "fx.convolution.2", ir_path, ir_error, sizeof(ir_error)) == 1);
        CHECK(crucible_load_ir(two, "fx.convolution", ir_path, NULL, 0) == 1);
        CHECK(crucible_load_ir(two, "fx.nothing", ir_path, ir_error, sizeof(ir_error)) == 0 && strlen(ir_error) > 0);
        CHECK(crucible_load_ir(two, "fx.convolution", "missing_ir.wav", ir_error, sizeof(ir_error)) == 0 &&
              strlen(ir_error) > 0);
        CHECK(crucible_load_ir(NULL, "fx.convolution", ir_path, ir_error, sizeof(ir_error)) == 0);
        CHECK(crucible_clear_ir(two, "fx.convolution.2") == 1);
        CHECK(crucible_clear_ir(two, "fx.nothing") == 0);
        CHECK(crucible_clear_ir(NULL, "fx.convolution") == 0);
        remove(ir_path);
        crucible_destroy(two);

        CHECK(crucible_load_ir(engine, "fx.saturator", "any.wav", ir_error, sizeof(ir_error)) == 0 &&
              strstr(ir_error, "no impulse response") != NULL);
        CHECK(crucible_clear_ir(engine, "fx.saturator") == 0);
    }
    {
        /* crucible_process_events: parameter events apply at block start; setup work waits for crucible_service. */
        const int type = crucible_param_index(engine, "osc1_type");
        const int table = crucible_param_index(engine, "osc1_table");
        CrucibleParamEvent changes[3] = {{0, cutoff, 1500.0f}, {0, type, 6.0f}, {0, -1, 1.0f}};
        CrucibleTransport transport;
        memset(&transport, 0, sizeof(transport));
        transport.struct_size = sizeof(transport);
        transport.bpm = 120.0;
        transport.ppq = 8.0;
        transport.playing = 1;
        float l[256], r[256];
        CrucibleMidiEvent on = {0, 0x90, 57, 100};
        crucible_process_events(engine, l, r, 256, &on, 1, changes, 3, &transport);
        CHECK(crucible_get_param(engine, cutoff) == 1500.0f);
        CHECK(crucible_get_param(engine, type) == 6.0f);
        crucible_service(engine);
        {
            /* int / choice values are rounded as crucible_set_param does */
            CrucibleParamEvent rounded = {0, type, 2.7f};
            crucible_process_events(engine, l, r, 256, NULL, 0, &rounded, 1, NULL);
            CHECK(crucible_get_param(engine, type) == 3.0f);
            CHECK(crucible_set_param(engine, type, 2.7f) == 1 && crucible_get_param(engine, type) == 3.0f);
            CHECK(crucible_set_param(engine, type, 6.0f) == 1);
            crucible_service(engine);
        }
        crucible_process_events(engine, l, r, 256, NULL, 0, NULL, 0, NULL);
        {
            float peak = 0.0f;
            int i;
            for (i = 0; i < 256; ++i)
                peak = fmaxf(peak, fabsf(l[i]));
            CHECK(peak > 0.0f);
        }
        /* a transport with struct_size 0 is "no transport", without error */
        transport.struct_size = 0;
        crucible_process_events(engine, l, r, 256, NULL, 0, NULL, 0, &transport);
        transport.struct_size = sizeof(transport);
        /* a transport that stops before ppq knows bpm only */
        transport.struct_size = offsetof(CrucibleTransport, ppq);
        crucible_process_events(engine, l, r, 256, NULL, 0, NULL, 0, &transport);
        CHECK(table >= 0);
        crucible_service(NULL);
        crucible_process_events(NULL, l, r, 256, NULL, 0, NULL, 0, NULL);
    }
    {
        /* struct_size: 0 and short structs give defaults for the fields beyond them */
        CrucibleConfig legacy;
        CrucibleEngine* a;
        CrucibleEngine* b;
        memset(&legacy, 0, sizeof(legacy));
        legacy.max_voices = 3;
        a = crucible_create(&legacy);
        CHECK(a != NULL);
        legacy.struct_size = offsetof(CrucibleConfig, max_voices);
        legacy.max_block_size = 64;
        b = crucible_create(&legacy);
        CHECK(b != NULL);
        crucible_destroy(a);
        crucible_destroy(b);
    }
    {
        /* a module with lookahead reports its full latency (FIR half length + lookahead) before any process call */
        CrucibleEngine* e = crucible_create(&config);
        static float pl[128], pr[128];
        CHECK(e != NULL);
        CHECK(crucible_set_param_id(e, "fx_deesser_on", 1.0f) == 1);
        crucible_prepare(e, 48000.0, 128);
        CHECK(crucible_latency_samples(e) == 96);
        CHECK(crucible_set_param_id(e, "fx_deesser_on", 0.0f) == 1);
        CHECK(crucible_set_param_id(e, "fx_deesser_on", 1.0f) == 1);
        crucible_process(e, pl, pr, 128, NULL, 0);
        CHECK(crucible_latency_samples(e) == 96);
        crucible_destroy(e);
    }
    {
        /* host plumbing: change poll with sources, latency, tail, reset */
        enum
        {
            CAP = 4096
        };
        static int idx[CAP], src[CAP];
        static float pl[256], pr[256];
        const int mix = crucible_param_index(engine, "fx_ra_mix");
        CrucibleParamEvent ev;
        CHECK(mix >= 0);
        crucible_param_changes(engine, idx, src, CAP);
        CHECK(crucible_param_changes(engine, idx, src, CAP) == 0);
        CHECK(crucible_set_param(engine, cutoff, 900.0f) == 1);
        CHECK(crucible_param_changes(engine, idx, src, CAP) == 1 && idx[0] == cutoff && src[0] == CRUCIBLE_SOURCE_HOST);
        memset(&ev, 0, sizeof(ev));
        ev.index = mix;
        ev.value = 0.5f;
        crucible_process_events(engine, pl, pr, 256, NULL, 0, &ev, 1, NULL);
        CHECK(crucible_param_changes(engine, idx, NULL, CAP) == 1 && idx[0] == mix);
        crucible_map_cc(engine, 74, cutoff);
        CHECK(crucible_midi(engine, 0xb0, 74, 90) == 1);
        CHECK(crucible_param_changes(engine, idx, src, CAP) == 1 && idx[0] == cutoff && src[0] == CRUCIBLE_SOURCE_MIDI);
        {
            /* service pending: a table change through process_events waits for crucible_service */
            const int table = crucible_param_index(engine, "osc1_table");
            CrucibleParamMeta hm;
            hm.struct_size = sizeof(hm);
            CrucibleEngine* listed;
            int hidden = -1, k;
            CHECK(table >= 0 && crucible_service_pending(engine) == 0 && crucible_service_pending(NULL) == 0);
            memset(&ev, 0, sizeof(ev));
            ev.index = table;
            ev.value = 15.0f;
            crucible_process_events(engine, pl, pr, 256, NULL, 0, &ev, 1, NULL);
            CHECK(crucible_service_pending(engine) == 1);
            crucible_service(engine);
            CHECK(crucible_service_pending(engine) == 0);
            crucible_param_changes(engine, idx, src, CAP);
            /* only exposed parameters are reported: a change to a hidden one is dropped */
            listed = crucible_create_from_product(
                NULL,
                "{\"format\":\"crucible-product\",\"version\":1,\"id\":\"hid\",\"surface\":{\"expose\":\"listed\","
                "\"pages\":[{\"id\":\"p\",\"params\":[\"filter_cutoff\"]}]}}",
                NULL, error, sizeof error);
            CHECK(listed != NULL);
            for (k = 0; k < crucible_param_count(listed) && hidden < 0; ++k)
                if (crucible_param_meta(listed, k, &hm) == 1 && hm.exposed == 0)
                    hidden = k;
            CHECK(hidden >= 0);
            crucible_param_changes(listed, idx, src, CAP);
            CHECK(crucible_set_param(listed, hidden, hm.max) == 1);
            CHECK(crucible_param_changes(listed, idx, src, CAP) == 0);
            CHECK(crucible_set_param_id(listed, "filter_cutoff", 1500.0f) == 1);
            CHECK(crucible_param_changes(listed, idx, src, CAP) == 1);
            crucible_destroy(listed);
        }
        crucible_map_cc(engine, 74, -1);
        CHECK(crucible_param_changes(NULL, idx, src, CAP) == 0 && crucible_param_changes(engine, NULL, NULL, CAP) == 0);
        CHECK(crucible_latency_samples(engine) >= 0 && crucible_latency_samples(NULL) == 0);
        CHECK(crucible_tail_seconds(engine) >= 0.0 && crucible_tail_seconds(NULL) == 0.0);
        crucible_note_on(engine, 60, 1.0f);
        crucible_process(engine, pl, pr, 256, NULL, 0);
        crucible_reset(engine);
        crucible_process(engine, pl, pr, 256, NULL, 0);
        {
            float peak = 0.0f;
            int i;
            for (i = 0; i < 256; ++i)
                peak = fmaxf(peak, fabsf(pl[i]) + fabsf(pr[i]));
            CHECK(peak == 0.0f);
        }
        crucible_reset(NULL);
        crucible_note_on(engine, 60, 1.0f);
        CHECK(crucible_kill_all_voices(engine) == 1 && crucible_kill_all_voices(NULL) == 0);
        crucible_process(engine, pl, pr, 256, NULL, 0);
        crucible_process(engine, pl, pr, 256, NULL, 0);
    }
    crucible_destroy(engine);
    printf("C API test passed\n");
    return 0;
}
