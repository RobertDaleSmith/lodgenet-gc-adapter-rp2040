// LodgeNet SNES/N64/GC controller adapter
// Protocol reference: Nielk1/LodgeNetController-USB
// https://github.com/Nielk1/LodgeNetController-USB

#include "adapter_includes.h"
#include "hardware/pio.h"
#include "generated/lodgenet.pio.h"

#define LED_PIN 25
#define CLK_PIN 3
#define P1_IN_PIN 2
#define VCC_PIN 4
#define CLK2_PIN 5
// GPIO 6 = floating (IR remote)

// Single PIO SM — switch programs for different protocols
static PIO pio_hw = pio0;
static uint sm = 0;
static uint current_offset = 0;
static const pio_program_t *current_program = NULL;

typedef enum {
    PROTO_NONE,
    PROTO_SR,    // SNES shift register
    PROTO_MCU,   // N64/GC microcontroller
} proto_mode_t;

static proto_mode_t proto = PROTO_NONE;

joybus_input_s _port_joybus[4] = {0, 0, 0, 0};

// Load MCU protocol (N64/GC) onto the SM
static void load_mcu_protocol(void)
{
    pio_sm_set_enabled(pio_hw, sm, false);
    if(current_program)
        pio_remove_program(pio_hw, current_program, current_offset);
    current_offset = pio_add_program(pio_hw, &lodgenet_mcu_program);
    current_program = &lodgenet_mcu_program;
    lodgenet_mcu_pio_init(pio_hw, sm, current_offset, CLK_PIN, P1_IN_PIN);
    proto = PROTO_MCU;
}

// Load SR protocol (SNES) onto the SM
static void load_sr_protocol(void)
{
    pio_sm_set_enabled(pio_hw, sm, false);
    if(current_program)
        pio_remove_program(pio_hw, current_program, current_offset);
    current_offset = pio_add_program(pio_hw, &lodgenet_sr_program);
    current_program = &lodgenet_sr_program;
    lodgenet_sr_pio_init(pio_hw, sm, current_offset, CLK_PIN, CLK2_PIN, P1_IN_PIN);
    proto = PROTO_SR;
}

// Read N bytes via MCU protocol PIO
static bool mcu_read(uint8_t *bytes, uint num_bytes, bool *is_gc)
{
    while(!pio_sm_is_rx_fifo_empty(pio_hw, sm))
        pio_sm_get(pio_hw, sm);

    pio_sm_put(pio_hw, sm, num_bytes * 8 - 1);

    uint32_t timeout_us = num_bytes * 8 * 50 + 5000;
    uint32_t start = time_us_32();

    for(uint i = 0; i < num_bytes; i++)
    {
        while(pio_sm_is_rx_fifo_empty(pio_hw, sm))
        {
            if((time_us_32() - start) > timeout_us)
            {
                *is_gc = false;
                return false;
            }
        }
        bytes[i] = (uint8_t)(pio_sm_get(pio_hw, sm));
    }

    bool has_mcu = (bytes[1] & 0x80) != 0;
    *is_gc = (bytes[1] & 0x40) != 0;
    bool forced_fail = *is_gc && (bytes[1] & 0x01);
    return has_mcu && !forced_fail;
}

// Read 16 bits + 1 presence bit via SR protocol PIO
static bool sr_read(uint16_t *value)
{
    while(!pio_sm_is_rx_fifo_empty(pio_hw, sm))
        pio_sm_get(pio_hw, sm);

    pio_sm_put(pio_hw, sm, 15);

    uint32_t timeout_us = 5000;
    uint32_t start = time_us_32();

    uint8_t raw[3] = {0};
    for(int i = 0; i < 3; i++)
    {
        while(pio_sm_is_rx_fifo_empty(pio_hw, sm))
        {
            if((time_us_32() - start) > timeout_us)
                return false;
        }
        raw[i] = (uint8_t)(pio_sm_get(pio_hw, sm));
    }

    *value = ((uint16_t)raw[0] << 8) | raw[1];

    // Presence bit: LOW = controller present, HIGH = no controller
    bool present = !(raw[2] & 0x01);
    return present;
}

static void clear_output(void)
{
    _port_joybus[0].button_a = 0;
    _port_joybus[0].button_b = 0;
    _port_joybus[0].button_x = 0;
    _port_joybus[0].button_y = 0;
    _port_joybus[0].button_z = 0;
    _port_joybus[0].button_r = 0;
    _port_joybus[0].button_l = 0;
    _port_joybus[0].button_start = 0;
    _port_joybus[0].dpad_up = 0;
    _port_joybus[0].dpad_down = 0;
    _port_joybus[0].dpad_left = 0;
    _port_joybus[0].dpad_right = 0;
    _port_joybus[0].stick_left_x = 127;
    _port_joybus[0].stick_left_y = 127;
    _port_joybus[0].stick_right_x = 127;
    _port_joybus[0].stick_right_y = 127;
    _port_joybus[0].analog_trigger_l = 0;
    _port_joybus[0].analog_trigger_r = 0;
}

static uint8_t last_dpad = 0;
static uint8_t last_menu = 0;

static void parse_mcu(uint8_t *bytes, bool is_gc)
{
    clear_output();

    _port_joybus[0].button_z     = !(bytes[0] & 0x20);
    _port_joybus[0].button_start = !(bytes[0] & 0x10);
    _port_joybus[0].button_l     = !(bytes[1] & 0x20);
    _port_joybus[0].button_r     = !(bytes[1] & 0x10);

    // Raw dpad (active LOW, inverted): bits 3=U, 2=D, 1=L, 0=R
    uint8_t dpad = ~bytes[0] & 0x0F;

    // Detect LodgeNet encoded buttons (impossible SOCD combos)
    uint8_t encoded_type = 0;
    if((dpad & 0x03) == 0x03 || (dpad & 0x0C) == 0x0C)
    {
        // Impossible combo detected
        if(last_dpad == 0)
        {
            if(dpad == 0x0F) encoded_type = 1;  // Reset: all 4
            if(dpad == 0x0C) encoded_type = 2;  // Menu: U+D
            if(dpad == 0x03) encoded_type = 3;  // *: L+R
            if(dpad == 0x0D) encoded_type = 4;  // Select: U+D+R
            if(dpad == 0x0B) encoded_type = 5;  // Order: U+L+R
            if(dpad == 0x0E) encoded_type = 6;  // #: U+D+L
            if(last_menu == 0)
                last_menu = encoded_type;
            else if(last_menu != encoded_type)
                encoded_type = last_menu;
        }
    }
    else
    {
        last_dpad = dpad;
        last_menu = 0;
    }

    // Use real dpad (not encoded buttons)
    _port_joybus[0].dpad_up      = (last_dpad >> 3) & 1;
    _port_joybus[0].dpad_down    = (last_dpad >> 2) & 1;
    _port_joybus[0].dpad_left    = (last_dpad >> 1) & 1;
    _port_joybus[0].dpad_right   = (last_dpad >> 0) & 1;

    // LodgeNet encoded system buttons (limited by stock xinput.c mappings)
    // Encoded buttons detected but no stock XInput route for Back/Guide
    (void) encoded_type;

    if(is_gc)
    {
        _port_joybus[0].button_a     = !(bytes[0] & 0x40);
        _port_joybus[0].button_b     = !(bytes[0] & 0x80);
        _port_joybus[0].button_x     = !(bytes[1] & 0x04);
        _port_joybus[0].button_y     = !(bytes[1] & 0x08);
        _port_joybus[0].stick_left_x     = bytes[2];
        _port_joybus[0].stick_left_y     = 255 - bytes[3];
        _port_joybus[0].stick_right_x    = bytes[4];
        _port_joybus[0].stick_right_y    = 255 - bytes[5];
        _port_joybus[0].analog_trigger_l = bytes[6];
        _port_joybus[0].analog_trigger_r = bytes[7];
    }
    else
    {
        _port_joybus[0].button_a     = !(bytes[0] & 0x80);  // A
        _port_joybus[0].button_b     = !(bytes[0] & 0x40);  // B
        // C-buttons: no L3/R3 in stock xinput, map all 4 to face buttons
        _port_joybus[0].button_x     = !(bytes[1] & 0x08);  // C-Up → X
        _port_joybus[0].button_y     = !(bytes[1] & 0x04);  // C-Down → Y
        // C-Left/C-Right: no stock route, use right stick digital
        bool c_left  = !(bytes[1] & 0x02);
        bool c_right = !(bytes[1] & 0x01);
        _port_joybus[0].stick_right_x = c_right ? 255 : (c_left ? 0 : 127);

        // N64 analog: signed ±80 range, scale to 0-255
        int8_t raw_x = (int8_t)bytes[2];
        int8_t raw_y = (int8_t)bytes[3];
        // Clamp to ±80 then scale: (val + 80) * 255 / 160
        if(raw_x > 80) raw_x = 80;
        if(raw_x < -80) raw_x = -80;
        if(raw_y > 80) raw_y = 80;
        if(raw_y < -80) raw_y = -80;
        _port_joybus[0].stick_left_x = (uint8_t)((raw_x + 80) * 255 / 160);
        _port_joybus[0].stick_left_y = (uint8_t)((raw_y + 80) * 255 / 160);
    }
}

static void parse_snes(uint16_t value)
{
    // LODG: M O B Y S * ↑ ↓ 1 1 ← → A X L R
    //       15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0
    clear_output();

    _port_joybus[0].button_a     = !(value & 0x2000);  // B → A
    _port_joybus[0].button_b     = !(value & 0x0008);  // A → B
    _port_joybus[0].button_x     = !(value & 0x1000);  // Y → X
    _port_joybus[0].button_y     = !(value & 0x0004);  // X → Y
    _port_joybus[0].button_z     = !(value & 0x0800);  // Select → RB
    _port_joybus[0].button_start = !(value & 0x0400);  // Start
    _port_joybus[0].button_l     = !(value & 0x0002);  // L → LT
    _port_joybus[0].button_r     = !(value & 0x0001);  // R → RT

    // D-pad with SOCD detection
    bool raw_up    = !(value & 0x0200);
    bool raw_down  = !(value & 0x0100);
    bool raw_left  = !(value & 0x0020);
    bool raw_right = !(value & 0x0010);

    bool ln_minus = raw_up && raw_down;
    bool ln_plus  = raw_left && raw_right;

    if(ln_minus)
    {
        _port_joybus[0].dpad_up = 0;
        _port_joybus[0].dpad_down = 0;
    }
    else
    {
        _port_joybus[0].dpad_up = raw_up;
        _port_joybus[0].dpad_down = raw_down;
    }

    if(ln_plus)
    {
        _port_joybus[0].dpad_left = 0;
        _port_joybus[0].dpad_right = 0;
    }
    else
    {
        _port_joybus[0].dpad_left = raw_left;
        _port_joybus[0].dpad_right = raw_right;
    }

    // LodgeNet system buttons detected but no stock XInput route
    // Menu = !(value & 0x8000), Order = !(value & 0x4000)
    // Minus (U+D SOCD) and Plus (L+R SOCD) cleaned from dpad above
}

bool cb_adapter_hardware_test()
{
    return true;
}

void joybus_itf_poll(joybus_input_s **out)
{
    *out = _port_joybus;
    _port_joybus[0].port_itf = 0;

    // Throttle MCU (N64/GC) to 16ms. SNES (SR) polls as fast as framework allows.
    static uint32_t last_poll = 0;
    uint32_t now = time_us_32();
    if(proto != PROTO_SR && (now - last_poll < 16000))
        return;
    last_poll = now;

    static uint8_t fail_count = 0;
    static uint8_t good_count = 0;

    if(proto == PROTO_SR)
    {
        uint16_t snes_value = 0;
        bool snes_present = sr_read(&snes_value);

        if(snes_present)
        {
            fail_count = 0;
            parse_snes(snes_value);
        }
        else if(++fail_count >= 5)
        {
            clear_output();
            load_mcu_protocol();
            fail_count = 0;
        }
        return;
    }

    // Default: try MCU (N64/GC)
    {
        uint8_t bytes[10];
        bool is_gc = false;
        bool valid = mcu_read(bytes, 10, &is_gc);

        if(valid)
        {
            proto = PROTO_MCU;
            fail_count = 0;
            if(++good_count >= 15)
                parse_mcu(bytes, is_gc);
        }
        else
        {
            good_count = 0;  // any bad read resets the good streak
            if(++fail_count >= 5)
            {
                clear_output();
                load_sr_protocol();
                fail_count = 0;
            }
        }
    }
}

void joybus_itf_enable_rumble(uint8_t interface, bool enable)
{
    (void) interface;
    (void) enable;
}

void rgb_itf_update(rgb_s *leds)
{
    // nothing
}

void joybus_itf_init()
{
    gpio_init(CLK_PIN);
    gpio_pull_up(CLK_PIN);
    gpio_set_dir(CLK_PIN, GPIO_OUT);
    gpio_put(CLK_PIN, 1);

    gpio_init(P1_IN_PIN);
    gpio_pull_up(P1_IN_PIN);
    gpio_set_dir(P1_IN_PIN, GPIO_IN);

    gpio_init(VCC_PIN);
    gpio_set_dir(VCC_PIN, GPIO_OUT);
    gpio_put(VCC_PIN, 1);

    gpio_init(CLK2_PIN);
    gpio_pull_up(CLK2_PIN);
    gpio_set_dir(CLK2_PIN, GPIO_OUT);
    gpio_put(CLK2_PIN, 1);

    load_mcu_protocol();
}

void rgb_itf_init()
{
    gpio_init(LED_PIN);
    gpio_pull_up(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
}

void main()
{
    adapter_main_init();
    adapter_main_loop();
}
