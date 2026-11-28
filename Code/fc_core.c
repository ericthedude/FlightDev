#include "fc.h"
#include <string.h>

#define ROT_NONE {{1,0,0},{0,1,0},{0,0,1}}
#define ROT_ROLL_180 {{1,0,0},{0,-1,0},{0,0,-1}}
#define ROT_YAW_90 {{0,-1,0},{1,0,0},{0,0,1}}

typedef struct { float roll, pitch, yaw, throttle; bool idle_only; } ctrl_cmd_t;
typedef enum { MIX_CONVENTIONAL = 0, MIX_ELEVON} plane_mix_t;

typedef struct vehicle_profile vehicle_profile_t;
struct vehicle_profile {
    const char *name;
    float imu_rot[3][3];
    float gyro_lpf_hz, dterm_lpf_hz;
    pid_gains_t pid[3];
    float max_rate_dps[3], expo[3];
    float angle_p, max_angle_deg;
    float est_kp, est_gate_g;
    float tpa, tpa_break;
    float idle_throttle;
    uint32_t fs_delay_ms;
    uint32_t fs_cut_ms;
    float fs_throttle;
    plane_mix_t plane_mix;
    float servo_trim[4];
    bool servo_reverse[4];
    void (*init_outputs)(void);
    void (*control)(const vehicle_profile_t *p, ctrl_cmd_t *output);
    void (*mix_and_write)(const vehicle_profile_t *p, const ctrl_cmd_t *c);
};
static void control_drone(const vehicle_profile_t *p, ctrl_cmd_t *output);
static void control_plane(const vehicle_profile_t *p, ctrl_cmd_t *output);
static void mix_drone(const vehicle_profile_t *p, const ctrl_cmd_t *c);
static void mix_plane(const vehicle_profile_t *p, const ctrl_cmd_t *c);

static const vehicle_profile_t PROFILES[2] = {
    [MODE_DRONE] = {
        .name = "Drone",
        .imu_rot = ROT_NONE,
        .gyro_lpf_hz = 150.0f, .dterm_lpf_hz = 100.0f,
        .pid = { {0.0020f, 0.0030f, 0.00030f, 0.25f}, 
                 {0.0020f, 0.0030f, 0.00030f, 0.25f},
                 {0.0030f, 0.0030f, 0.25f} },
        .max_rate_dps = {700.0f, 700.0f, 600.0f}, .expo = {0.3f, 0.3f, 0.3f},
        .angle_p = 6.0f, .max_angle_deg = 45.0f,
        .est_kp = 0.8f, .est_gate_g = 0.40f,
        .tpa = 0.0f, .tpa_break = 0.5f,
        .idle_throttle = 0.06f,
        .fs_delay_ms = 500, .fs_cut_ms = 0, .fs_throttle = 0.0f,
        .init_outputs = hal_outputs_init_drone,
        .control = control_drone, .mix_and_write = mix_drone,
    },
    [MODE_PLANE] = {
        .name = "Plane",
        .imu_rot = ROT_ROLL_180,
        .gyro_lpf_hz = 40.0f, .dterm_lpf_hz = 30.0f,
        .pid = { {0.0080f, 0.0100f, 0.0f, 0.30f}, 
                 {0.0080f, 0.0100f, 0.0f, 0.30f},
                 {0.0050f, 0.0050f, 0.0f, 0.30f} },
        .max_rate_dps = {150.0f, 90.0f, 60.0f}, .expo = {0.2f, 0.2f, 0.2f},
        .angle_p = 2.0f, .max_angle_deg = 45.0f,
        .est_kp = 0.3f, .est_gate_g = 0.15f,
        .tpa = 0.03f, .tpa_break = 0.6f,
        .idle_throttle = 0.0f,
        .fs_delay_ms = 0, .fs_cut_ms = 20000, .fs_throttle = 0.30f,
        .plane_mix = MIX_CONVENTIONAL,
        .servo_trim = {0, 0, 0, 0}, .servo_reverse = {false, false, false, false},
        .init_outputs = hal_outputs_init_plane,
        .control = control_plane, .mix_and_write = mix_plane,
    },
};

static struct {
    vehicle_mode_t mode;
    const vehicle_profile_t *prof;

    crsf_t crsf;
    rc_t rc;

    est_t est;
    biquad_t gyro_lpf[3];
    pid_ctl_t pid[3];
    float gyro_dps[3];

    vec3_t bias_sum, gyro_bias;
    int calib_n;
    bool calibrated;
    int imu_fail;
    bool imu_ok;
    bool armed;
    bool failsafe;
    bool arm_seen_low;
    bool initialized;
    int settle_left;

    uint32_t t_us;
    uint32_t mode_diff_since;
} S = {
    .mode = MODE_DRONE,
    .prof = &PROFILES[MODE_DRONE],
};

static float norm_ch(uint16_t v) { return clampf(((float)v - 992.0f) / 819.0f, -1.0f, 1.0f); }
static float deadband(float x, float d)
{
    return fabsf(x) < d ? 0.0f : x;
}


static void pids_reset(void)
{
    for (int i = 0; i < 3; i++) pid_init(&S.pid[i], &S.prof->pid[i], S.prof->dterm_lpf_hz, LOOP_DT);
}

static void set_mode(vehicle_mode_t m)
{
    hal_outputs_disable_all();
    S.mode = m;
    S.prof = &PROFILES[m];
    for (int i = 0; i < 3; i++) biquad_lpf_init(&S.gyro_lpf[i], S.prof->gyro_lpf_hz, (float)LOOP_HZ);
    pids_reset();
    est_init(&S.est);
    S.settle_left = SETTLE_SAMPLES;
    S.prof->init_outputs();
    hal_beep(m == MODE_PLANE ? BEEP_PLANE : BEEP_DRONE);
    hal_log("MODE -> %s\n", S.prof->name);
}

static void disarm(const char *why)
{
    if (!S.armed) return;
    S.armed = false;
    pids_reset();
    hal_beep(BEEP_DISARMED);
    hal_log("DISARMED (%s)\n", why);
}

static float rate_curve(float x, float max_dps, float expo)
{
    return max_dps * ((1.0f - expo) * x + expo * x * x * x);
}

static float tpa_scale(const vehicle_profile_t *p, float thr)
{
    if (thr <= p->tpa_break) return 1.0f;
    return 1.0f - p->tpa * (thr - p->tpa_break) / (1.0f - p->tpa_break);
}

static void axes_setpoints(const vehicle_profile_t *p, flight_mode_t fm, float r, float pch, float y, float sp[3])
{
    if (fm == FM_ANGLE) {
        const float er = r  * p->max_angle_deg - S.est.roll * RAD2DEG;
        const float ep = pch * p->max_angle_deg - S.est.pitch * RAD2DEG;
        sp[0] = clampf(p->angle_p * er, -p->max_rate_dps[0], p->max_rate_dps[0]);
        sp[1] = clampf(p->angle_p * ep, -p->max_rate_dps[1], p->max_rate_dps[1]);

    } else {
        sp[0] = rate_curve(r, p->max_rate_dps[0], p->expo[0]);
        sp[1] = rate_curve(pch, p->max_rate_dps[1], p->expo[1]);
    }
    sp[2] = rate_curve(y, p->max_rate_dps[2], p->expo[2]);
}

static void rate_loop(const float sp[3], float tpa, bool i_en, float out[3])
{
    for (int i = 0; i < 3; i++)
        out[i] = clampf(pid_update(&S.pid[i], sp[i], S.gyro_dps[i], LOOP_DT, tpa, i_en), -1.0f, 1.0f);
}

static void control_drone(const vehicle_profile_t *p, ctrl_cmd_t *out)
{
    const flight_mode_t fm = (S.rc.fmode_sw == 0) ? FM_RATE : FM_ANGLE;
    const float thr = S.rc.throttle;
    float sp[3], r[3];

    axes_setpoints(p, fm, S.rc.roll, S.rc.pitch, S.rc.yaw, sp);
    out->idle_only = thr < 0.03f;
    rate_loop(sp, tpa_scale(p, thr), thr > 0.10f, r);
    out->roll = r[0]; out->pitch = r[1]; out->yaw = r[2];
    out->throttle = p->idle_throttle + thr * (1.0f - p->idle_throttle);
}

static void control_plane(const vehicle_profile_t *p, ctrl_cmd_t *out)
{
    flight_mode_t fm = (S.rc.fmode_sw == 0) ? FM_MANUAL : ((S.rc.fmode_sw == 1) ? FM_RATE : FM_ANGLE);
    float r_in = S.rc.roll, p_in = S.rc.pitch, y_in = S.rc.yaw, thr = S.rc.throttle;

    out->idle_only = false;
    if (S.failsafe) {
        const uint32_t lost_ms = (S.t_us - S.crsf.last_frame_us) / 1000u;
        fm = FM_ANGLE;
        r_in = p_in = y_in = 0.0f;
        thr = (lost_ms < p->fs_cut_ms) ? p->fs_throttle : 0.0f;
    }
    out->throttle = thr;
    if (fm == FM_MANUAL) {
        out->roll = r_in; out->pitch = p_in; out->yaw = y_in;
        pids_reset();
        return;
    }
    float sp[3], r[3];
    axes_setpoints(p, fm, r_in, p_in, y_in, sp);
    rate_loop(sp, tpa_scale(p, thr), true, r);
    out->roll = r[0]; out->pitch = r[1]; out->yaw = r[2];
}


static const float MIX_R[4] = { -1.0f, -1.0f, +1.0f, +1.0f };
static const float MIX_P[4] = { +1.0f, -1.0f, -1.0f, +1.0f };
static const float MIX_Y[4] = { +1.0f, -1.0f, +1.0f, -1.0f };
static void mix_drone(const vehicle_profile_t *p, const ctrl_cmd_t *c)
{
    float m[4];
    if (c->idle_only) {
        for (int i = 0; i < 4; i++) m[i] = p->idle_throttle;
    } else {
        float hi = -1e9f;
        for (int i = 0; i < 4; i++) {
            m[i] = c->throttle + c->roll * MIX_R[i] + c->pitch * MIX_P[i] + c->yaw * MIX_Y[i];
            if (m[i] > hi) hi = m[i];
        }
        if (hi > 1.0f)
            for (int i = 0; i < 4; i++) m[i] -= hi - 1.0f;
    }
    for (int i = 0; i < 4; i++) m[i] = clampf(m[i], p->idle_throttle, 1.0f);
    hal_motor_write(m);
}

static void mix_plane(const vehicle_profile_t *p, const ctrl_cmd_t *c)
{
    float s[4] = {0, 0, 0, 0};
    if (p->plane_mix == MIX_ELEVON) {
        s[0] = -c->pitch + c->roll;
        s[1] = -c->pitch - c->roll;
    } else {
        s[0] = c->roll;
        s[1] = -c->roll;
        s[2] = -c->pitch;
        s[3] = c->yaw;
    }
    for (int i = 0; i<4; i++) {
        if (p->servo_reverse[i]) s[i] = -s[i];
        s[i] = clampf(s[i] + p->servo_trim[i], -1.0f, 1.0f);
    }
    hal_servo_write(s, clampf(c->throttle, 0.0f, 1.0f));
}

static void write_disarmed(void)
{
    if (S.mode == MODE_DRONE) {
        const float z[4] = {0, 0, 0, 0};
        hal_motor_write(z);
    } else {
        ctrl_cmd_t c = {0};
        if (S.rc.valid) { c.roll = S.rc.roll; c.pitch = S.rc.pitch; c.yaw = S.rc.yaw; }
        mix_plane(S.prof, &c);
    }
}


static void mode_task(void)
{
    if (S.armed || !S.rc.valid) { S.mode_diff_since = S.t_us; return; }

    const vehicle_mode_t req = S.rc.plane_sw? MODE_PLANE : MODE_DRONE;
    if (req == S.mode) { S.mode_diff_since = S.t_us; return; }
    if (S.t_us - S.mode_diff_since >= MODE_DEBOUNCE_US) {
        set_mode(req);
        S.mode_diff_since = S.t_us;
    }
}

static void arming_task(void)
{
    if(!S.rc.arm_sw) S.arm_seen_low = true;

    if (S.armed) {
        if (!S.imu_ok) disarm("imu fault");
        else if (S.rc.valid && !S.rc.arm_sw) disarm("switch");
        return;
    }
    if (!(S.rc.valid && S.rc.arm_sw && S.arm_seen_low)) return;
    S.arm_seen_low = false;

    bool ok = S.calibrated && S.settle_left == 0 && S.imu_ok && S.rc.throttle < 0.05f;
    if (ok && S.mode == MODE_DRONE)
        ok = fabsf(S.est.roll) < 25.0f * DEG2RAD && fabsf(S.est.pitch) < 25.0f * DEG2RAD;
#if FC_USE_DOCK_DETECT
    if (ok) ok = (hal_dock_present() == (S.mode == MODE_PLANE));
#endif
    if (ok) {
        S.armed = true;
        pids_reset();
        hal_beep(BEEP_ARMED);
        hal_log("ARMED (%s)\n", S.prof->name);
    } else {
        hal_beep(BEEP_DENIED);
        hal_log("ARM DENIED\n");
    }
}

static void failsafe_task(void)
{
    S.failsafe = S.armed && !S.rc.valid;
    if (!S.failsafe) return;
    const uint32_t lost_ms = (S.t_us - S.crsf.last_frame_us) / 1000u;
    if (S.mode == MODE_DRONE && lost_ms >= S.prof ->fs_delay_ms) disarm("link lost");
}

static void calibrate(const imu_sample_t *r)
{
    const float mag = sqrtf(r->gyro.x * r->gyro.x + r->gyro.y * r->gyro.y + r->gyro.z * r->gyro.z) * RAD2DEG;
    if (mag > 30.0f) {
        S.calib_n = 0;
        S.bias_sum.x = S.bias_sum.y = S.bias_sum.z = 0.0f;
        return;
    }
    S.bias_sum.x += r->gyro.x; S.bias_sum.y += r->gyro.y; S.bias_sum.z += r->gyro.z;
    if (++S.calib_n >= CALIB_SAMPLES) {
        S.gyro_bias.x = S.bias_sum.x / (float)S.calib_n;
        S.gyro_bias.y = S.bias_sum.y / (float)S.calib_n;
        S.gyro_bias.z = S.bias_sum.z / (float)S.calib_n;
        S.calibrated = true;
        hal_log("gyro calibrated\n");
    }
}

void fc_step(void)
{
    if (!S.initialized) {
        set_mode(MODE_DRONE);
        S.initialized = true;
    }

    S.t_us = hal_micros();
    uint8_t b;
    while (hal_rc_read_byte(&b))
        if (crsf_feed(&S.crsf, b, S.t_us)) rc_update_from_crsf();
    S.rc.valid = S.crsf.have_frame && (S.t_us - S.crsf.last_frame_us) < RC_TIMEOUT_US;

    imu_sample_t raw;
    if(!hal_imu_read(&raw)) {
        if (++S.imu_fail > 20) { S.imu_ok = false; disarm("imu lost"); write_disarmed(); }
        hal_watchdog_kick();
        return;
    }
    S.imu_fail = 0;
    S.imu_ok = true;

    if(!S.calibrated) {
        calibrate(&raw);
        write_disarmed();
        hal_watchdog_kick();
        return;
    }

    const vec3_t gs = { raw.gyro.x - S.gyro_bias.x, raw.gyro.y - S.gyro_bias.y, raw.gyro.z - S.gyro_bias.z};
    const vec3_t g = rot_apply(S.prof->imu_rot, gs);
    const vec3_t a = rot_apply(S.prof->imu_rot, raw.accel);

    const bool settling = S.settle_left > 0;
    if (settling) S.settle_left--;
    est_update(&S.est, g, a, settling ? 8.0f : S.prof->est_kp, 0.0f,
            settling ? 0.5f : S.prof->est_gate_g, LOOP_DT);

    S.gyro_dps[0] = biquad_apply(&S.gyro_lpf[0], g.x * RAD2DEG);
    S.gyro_dps[1] = biquad_apply(&S.gyro_lpf[1], g.y * RAD2DEG);
    S.gyro_dps[2] = biquad_apply(&S.gyro_lpf[2], g.z * RAD2DEG);

    mode_task();
    arming_task();
    failsafe_task();

    if (S.armed) {
        ctrl_cmd_t cmd;
        S.prof->control(S.prof, &cmd);
        S.prof->mix_and_write(S.prof, &cmd);
    } else {
        write_disarmed();
    }
    hal_watchdog_kick();
}

void fc_get_status(fc_status_t *st)
{
    st->mode = S.mode;
    st->armed = S.armed;
    st->calibrated = S.calibrated;
    st->failsafe = S.failsafe;
    st->roll_deg = S.est.roll * RAD2DEG;
    st->pitch_deg = S.est.pitch * RAD2DEG;
}
