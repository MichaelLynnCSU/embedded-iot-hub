/******************************************************************************
 * \file test_doorbell_main.c
 * \brief Unit tests for ESP32 doorbell node pure logic.
 *
 * Coverage:
 *   cam_pack_header()   / cam_unpack_header()   cam_logic.h -- 20-byte
 *       network-order header pack/roundtrip (magic, version, device_id,
 *       event_id, jpeg_size)
 *   cam_header_valid()                          cam_logic.h -- magic check
 *   cam_device_id_valid()                       cam_logic.h -- range check
 *   cam_make_event_id()                         cam_logic.h -- bit-packed
 *       mac_tail(24b) | boot_ms(24b) | seq(16b) correlation key
 *   cam_pack_stream_frame_hdr()                 cam_logic.h -- 4-byte
 *       big-endian stream frame length header
 *   Network / mode config constants -- BBB alignment regression guards
 *   Camera pin constants -- silkscreen regression guards
 ******************************************************************************/

#include "unity.h"
#include "cam_logic.h"

void setUp(void)    {}
void tearDown(void) {}

/******************************************************************************
 * cam_pack_header / cam_unpack_header
 ******************************************************************************/

void test_pack_header_magic(void)
{
    uint8_t buf[CAM_HEADER_SIZE] = {0};
    cam_pack_header(buf, 0, 0, 0);
    uint32_t magic = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                      ((uint32_t)buf[2] <<  8) | (uint32_t)buf[3];
    TEST_ASSERT_EQUAL_UINT32(CAM_MAGIC, magic);
}

void test_pack_header_version(void)
{
    uint8_t buf[CAM_HEADER_SIZE] = {0};
    cam_pack_header(buf, 0, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(CAM_HEADER_VERSION, buf[4]);
}

void test_pack_header_device_id(void)
{
    uint8_t buf[CAM_HEADER_SIZE] = {0};
    cam_pack_header(buf, 3, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(3, buf[5]);
}

void test_pack_header_reserved_zero(void)
{
    uint8_t buf[CAM_HEADER_SIZE] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    cam_pack_header(buf, 0, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(0, buf[6]);
    TEST_ASSERT_EQUAL_UINT8(0, buf[7]);
}

void test_pack_header_event_id_msb_first(void)
{
    uint8_t buf[CAM_HEADER_SIZE] = {0};
    cam_pack_header(buf, 0, 0x0102030405060708ULL, 0);
    TEST_ASSERT_EQUAL_UINT8(0x01, buf[8]);
    TEST_ASSERT_EQUAL_UINT8(0x02, buf[9]);
    TEST_ASSERT_EQUAL_UINT8(0x03, buf[10]);
    TEST_ASSERT_EQUAL_UINT8(0x04, buf[11]);
    TEST_ASSERT_EQUAL_UINT8(0x05, buf[12]);
    TEST_ASSERT_EQUAL_UINT8(0x06, buf[13]);
    TEST_ASSERT_EQUAL_UINT8(0x07, buf[14]);
    TEST_ASSERT_EQUAL_UINT8(0x08, buf[15]);
}

void test_pack_header_jpeg_size_msb_first(void)
{
    uint8_t buf[CAM_HEADER_SIZE] = {0};
    cam_pack_header(buf, 0, 0, 0x01020304UL);
    TEST_ASSERT_EQUAL_UINT8(0x01, buf[16]);
    TEST_ASSERT_EQUAL_UINT8(0x02, buf[17]);
    TEST_ASSERT_EQUAL_UINT8(0x03, buf[18]);
    TEST_ASSERT_EQUAL_UINT8(0x04, buf[19]);
}

void test_header_roundtrip_zero(void)
{
    uint8_t      buf[CAM_HEADER_SIZE] = {0};
    cam_header_t out;
    cam_pack_header(buf, 0, 0, 0);
    cam_unpack_header(buf, &out);
    TEST_ASSERT_EQUAL_UINT32(CAM_MAGIC, out.magic);
    TEST_ASSERT_EQUAL_UINT8(CAM_HEADER_VERSION, out.version);
    TEST_ASSERT_EQUAL_UINT8(0, out.device_id);
    TEST_ASSERT_EQUAL_UINT64(0, out.event_id);
    TEST_ASSERT_EQUAL_UINT32(0, out.jpeg_size);
}

void test_header_roundtrip_max(void)
{
    uint8_t      buf[CAM_HEADER_SIZE] = {0};
    cam_header_t out;
    cam_pack_header(buf, 0xFF, 0xFFFFFFFFFFFFFFFFULL, 0xFFFFFFFFUL);
    cam_unpack_header(buf, &out);
    TEST_ASSERT_EQUAL_UINT8(0xFF, out.device_id);
    TEST_ASSERT_EQUAL_UINT64(0xFFFFFFFFFFFFFFFFULL, out.event_id);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFUL, out.jpeg_size);
}

void test_header_roundtrip_known_values(void)
{
    uint8_t      buf[CAM_HEADER_SIZE] = {0};
    cam_header_t out;
    cam_pack_header(buf, 2, 0xDEADBEEFCAFEBABEULL, 9800UL);
    cam_unpack_header(buf, &out);
    TEST_ASSERT_EQUAL_UINT32(CAM_MAGIC, out.magic);
    TEST_ASSERT_EQUAL_UINT8(CAM_HEADER_VERSION, out.version);
    TEST_ASSERT_EQUAL_UINT8(2, out.device_id);
    TEST_ASSERT_EQUAL_UINT64(0xDEADBEEFCAFEBABEULL, out.event_id);
    TEST_ASSERT_EQUAL_UINT32(9800UL, out.jpeg_size);
}

void test_header_size_is_20(void)
{
    TEST_ASSERT_EQUAL_INT(20, CAM_HEADER_SIZE);
}

/******************************************************************************
 * cam_header_valid
 ******************************************************************************/

void test_header_valid_true_after_pack(void)
{
    uint8_t      buf[CAM_HEADER_SIZE] = {0};
    cam_header_t out;
    cam_pack_header(buf, 0, 1, 1);
    cam_unpack_header(buf, &out);
    TEST_ASSERT_EQUAL_INT(1, cam_header_valid(&out));
}

void test_header_valid_false_wrong_magic(void)
{
    cam_header_t h = {0};
    h.magic = 0x12345678UL;
    TEST_ASSERT_EQUAL_INT(0, cam_header_valid(&h));
}

/******************************************************************************
 * cam_device_id_valid
 ******************************************************************************/

void test_device_id_valid_zero(void)
{
    TEST_ASSERT_EQUAL_INT(1, cam_device_id_valid(0));
}

void test_device_id_valid_max_minus_one(void)
{
    /* MAX_DOORBELL_CAMS is 4 -> valid IDs are 0-3 */
    TEST_ASSERT_EQUAL_INT(1, cam_device_id_valid(MAX_DOORBELL_CAMS - 1));
}

void test_device_id_invalid_at_max(void)
{
    TEST_ASSERT_EQUAL_INT(0, cam_device_id_valid(MAX_DOORBELL_CAMS));
}

void test_device_id_invalid_well_above_max(void)
{
    TEST_ASSERT_EQUAL_INT(0, cam_device_id_valid(255));
}

/******************************************************************************
 * cam_make_event_id
 ******************************************************************************/

void test_make_event_id_zero(void)
{
    TEST_ASSERT_EQUAL_UINT64(0, cam_make_event_id(0, 0, 0));
}

void test_make_event_id_seq_only(void)
{
    /* seq occupies the low 16 bits untouched by mac_tail/boot_ms */
    TEST_ASSERT_EQUAL_UINT64(0x0042, cam_make_event_id(0, 0, 0x0042));
}

void test_make_event_id_mac_tail_masked_to_24_bits(void)
{
    /* mac_tail top byte must be masked off -- only lower 24 bits used */
    uint64_t id_masked   = cam_make_event_id(0x00FFFFFFUL, 0, 0);
    uint64_t id_unmasked = cam_make_event_id(0xFFFFFFFFUL, 0, 0);
    TEST_ASSERT_EQUAL_UINT64(id_masked, id_unmasked);
}

void test_make_event_id_boot_ms_masked_to_24_bits(void)
{
    uint64_t id_masked   = cam_make_event_id(0, 0x00FFFFFFUL, 0);
    uint64_t id_unmasked = cam_make_event_id(0, 0xFFFFFFFFUL, 0);
    TEST_ASSERT_EQUAL_UINT64(id_masked, id_unmasked);
}

void test_make_event_id_fields_dont_overlap(void)
{
    /* Changing only seq must not perturb mac_tail/boot_ms bits and vice
     * versa -- regression guard on the 24/24/16 bit layout. */
    uint64_t base    = cam_make_event_id(0x000001UL, 0x000000UL, 0);
    uint64_t bumped  = cam_make_event_id(0x000001UL, 0x000000UL, 1);
    TEST_ASSERT_EQUAL_UINT64(1, bumped - base);
}

void test_make_event_id_unique_across_devices(void)
{
    /* Two devices with different mac_tail, same boot_ms/seq, must not
     * collide -- this is the whole point of including mac_tail. */
    uint64_t id_a = cam_make_event_id(0x000001UL, 100, 5);
    uint64_t id_b = cam_make_event_id(0x000002UL, 100, 5);
    TEST_ASSERT_NOT_EQUAL(id_a, id_b);
}

/******************************************************************************
 * cam_pack_stream_frame_hdr
 ******************************************************************************/

void test_stream_hdr_zero(void)
{
    uint8_t buf[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    cam_pack_stream_frame_hdr(buf, 0);
    TEST_ASSERT_EQUAL_UINT8(0, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(0, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(0, buf[2]);
    TEST_ASSERT_EQUAL_UINT8(0, buf[3]);
}

void test_stream_hdr_msb_first(void)
{
    uint8_t buf[4] = {0};
    cam_pack_stream_frame_hdr(buf, 0x00000100UL);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(0x01, buf[2]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[3]);
}

void test_stream_hdr_known_value(void)
{
    uint8_t buf[4] = {0};
    cam_pack_stream_frame_hdr(buf, 4500UL);   /* typical QVGA JPEG size */
    uint32_t reconstructed = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                             ((uint32_t)buf[2] <<  8) | (uint32_t)buf[3];
    TEST_ASSERT_EQUAL_UINT32(4500UL, reconstructed);
}

/******************************************************************************
 * Network / mode config constants -- BBB alignment regression guards
 ******************************************************************************/

void test_bbb_host(void)
{
    TEST_ASSERT_EQUAL_STRING("10.0.1.1", BBB_HOST);
}

void test_bbb_port_snapshot(void)
{
    /* Snapshot mode -- doorbell cams share port 9091, separate from the
     * S3 PIR cam on 9090. */
    TEST_ASSERT_EQUAL_INT(9091, BBB_PORT);
}

void test_bbb_stream_port(void)
{
    TEST_ASSERT_EQUAL_INT(9093, BBB_STREAM_PORT);
}

void test_reconnect_ms(void)
{
    TEST_ASSERT_EQUAL_INT(3000, RECONNECT_MS);
}

void test_camera_manager_host(void)
{
    TEST_ASSERT_EQUAL_STRING("10.0.1.1", CAMERA_MANAGER_HOST);
}

void test_camera_manager_port(void)
{
    TEST_ASSERT_EQUAL_INT(9094, CAMERA_MANAGER_PORT);
}

void test_doorbell_mode_snapshot_value(void)
{
    TEST_ASSERT_EQUAL_INT(0, DOORBELL_MODE_SNAPSHOT);
}

void test_doorbell_mode_stream_value(void)
{
    TEST_ASSERT_EQUAL_INT(1, DOORBELL_MODE_STREAM);
}

void test_doorbell_mode_defaults_to_snapshot(void)
{
    /* Regression guard: if DOORBELL_MODE isn't overridden at build time,
     * it must default to SNAPSHOT, not STREAM. */
    TEST_ASSERT_EQUAL_INT(DOORBELL_MODE_SNAPSHOT, DOORBELL_MODE);
}

void test_max_doorbell_cams(void)
{
    TEST_ASSERT_EQUAL_INT(4, MAX_DOORBELL_CAMS);
}

void test_cam_magic_value(void)
{
    TEST_ASSERT_EQUAL_UINT32(0xCAFEBABEu, CAM_MAGIC);
}

void test_cam_header_version(void)
{
    TEST_ASSERT_EQUAL_INT(1, CAM_HEADER_VERSION);
}

void test_doorbell_id_default(void)
{
    TEST_ASSERT_EQUAL_INT(0, DOORBELL_ID);
}

/******************************************************************************
 * Button config
 ******************************************************************************/

void test_button_gpio(void)
{
    TEST_ASSERT_EQUAL_INT(13, BUTTON_GPIO);
}

void test_debounce_ms(void)
{
    TEST_ASSERT_EQUAL_INT(200, DEBOUNCE_MS);
}

/******************************************************************************
 * Camera pin constants -- AI-Thinker ESP32-CAM silkscreen regression guards
 ******************************************************************************/

void test_cam_pin_pwdn(void)  { TEST_ASSERT_EQUAL_INT(32, CAM_PIN_PWDN);  }
void test_cam_pin_reset(void) { TEST_ASSERT_EQUAL_INT(-1, CAM_PIN_RESET); }
void test_cam_pin_xclk(void)  { TEST_ASSERT_EQUAL_INT(0,  CAM_PIN_XCLK);  }
void test_cam_pin_siod(void)  { TEST_ASSERT_EQUAL_INT(26, CAM_PIN_SIOD);  }
void test_cam_pin_sioc(void)  { TEST_ASSERT_EQUAL_INT(27, CAM_PIN_SIOC);  }
void test_cam_pin_d7(void)    { TEST_ASSERT_EQUAL_INT(35, CAM_PIN_D7);    }
void test_cam_pin_d6(void)    { TEST_ASSERT_EQUAL_INT(34, CAM_PIN_D6);    }
void test_cam_pin_d5(void)    { TEST_ASSERT_EQUAL_INT(39, CAM_PIN_D5);    }
void test_cam_pin_d4(void)    { TEST_ASSERT_EQUAL_INT(36, CAM_PIN_D4);    }
void test_cam_pin_d3(void)    { TEST_ASSERT_EQUAL_INT(21, CAM_PIN_D3);    }
void test_cam_pin_d2(void)    { TEST_ASSERT_EQUAL_INT(19, CAM_PIN_D2);    }
void test_cam_pin_d1(void)    { TEST_ASSERT_EQUAL_INT(18, CAM_PIN_D1);    }
void test_cam_pin_d0(void)    { TEST_ASSERT_EQUAL_INT(5,  CAM_PIN_D0);    }
void test_cam_pin_vsync(void) { TEST_ASSERT_EQUAL_INT(25, CAM_PIN_VSYNC); }
void test_cam_pin_href(void)  { TEST_ASSERT_EQUAL_INT(23, CAM_PIN_HREF);  }
void test_cam_pin_pclk(void)  { TEST_ASSERT_EQUAL_INT(22, CAM_PIN_PCLK);  }

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_pack_header_magic);
    RUN_TEST(test_pack_header_version);
    RUN_TEST(test_pack_header_device_id);
    RUN_TEST(test_pack_header_reserved_zero);
    RUN_TEST(test_pack_header_event_id_msb_first);
    RUN_TEST(test_pack_header_jpeg_size_msb_first);
    RUN_TEST(test_header_roundtrip_zero);
    RUN_TEST(test_header_roundtrip_max);
    RUN_TEST(test_header_roundtrip_known_values);
    RUN_TEST(test_header_size_is_20);

    RUN_TEST(test_header_valid_true_after_pack);
    RUN_TEST(test_header_valid_false_wrong_magic);

    RUN_TEST(test_device_id_valid_zero);
    RUN_TEST(test_device_id_valid_max_minus_one);
    RUN_TEST(test_device_id_invalid_at_max);
    RUN_TEST(test_device_id_invalid_well_above_max);

    RUN_TEST(test_make_event_id_zero);
    RUN_TEST(test_make_event_id_seq_only);
    RUN_TEST(test_make_event_id_mac_tail_masked_to_24_bits);
    RUN_TEST(test_make_event_id_boot_ms_masked_to_24_bits);
    RUN_TEST(test_make_event_id_fields_dont_overlap);
    RUN_TEST(test_make_event_id_unique_across_devices);

    RUN_TEST(test_stream_hdr_zero);
    RUN_TEST(test_stream_hdr_msb_first);
    RUN_TEST(test_stream_hdr_known_value);

    RUN_TEST(test_bbb_host);
    RUN_TEST(test_bbb_port_snapshot);
    RUN_TEST(test_bbb_stream_port);
    RUN_TEST(test_reconnect_ms);
    RUN_TEST(test_camera_manager_host);
    RUN_TEST(test_camera_manager_port);
    RUN_TEST(test_doorbell_mode_snapshot_value);
    RUN_TEST(test_doorbell_mode_stream_value);
    RUN_TEST(test_doorbell_mode_defaults_to_snapshot);
    RUN_TEST(test_max_doorbell_cams);
    RUN_TEST(test_cam_magic_value);
    RUN_TEST(test_cam_header_version);
    RUN_TEST(test_doorbell_id_default);

    RUN_TEST(test_button_gpio);
    RUN_TEST(test_debounce_ms);

    RUN_TEST(test_cam_pin_pwdn);
    RUN_TEST(test_cam_pin_reset);
    RUN_TEST(test_cam_pin_xclk);
    RUN_TEST(test_cam_pin_siod);
    RUN_TEST(test_cam_pin_sioc);
    RUN_TEST(test_cam_pin_d7);
    RUN_TEST(test_cam_pin_d6);
    RUN_TEST(test_cam_pin_d5);
    RUN_TEST(test_cam_pin_d4);
    RUN_TEST(test_cam_pin_d3);
    RUN_TEST(test_cam_pin_d2);
    RUN_TEST(test_cam_pin_d1);
    RUN_TEST(test_cam_pin_d0);
    RUN_TEST(test_cam_pin_vsync);
    RUN_TEST(test_cam_pin_href);
    RUN_TEST(test_cam_pin_pclk);

    return UNITY_END();
}
