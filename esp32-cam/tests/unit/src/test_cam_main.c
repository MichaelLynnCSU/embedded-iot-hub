/******************************************************************************
 * \file test_cam_main.c
 * \brief Unit tests for ESP32-S3-CAM node pure logic.
 *
 * Coverage:
 *   cam_pack_header()   / cam_unpack_header()   cam_logic.h -- 20-byte
 *       network-order header pack/roundtrip (magic, version, device_id,
 *       event_id, jpeg_size)
 *   cam_header_valid()                          cam_logic.h -- magic check
 *   cam_device_id_valid()                       cam_logic.h -- range check
 *   cam_is_trigger()         cam_logic.h  -- UDP trigger string match
 *   Network config constants cam_logic.h  -- BBB port alignment regression
 *   Camera pin constants     cam_logic.h  -- silkscreen regression guards
 ******************************************************************************/

#include "unity.h"
#include "cam_logic.h"

void setUp(void)    {}
void tearDown(void) {}

/******************************************************************************
 * JPEG header packing
 ******************************************************************************/

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
    cam_pack_header(buf, 2, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(2, buf[5]);
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
    /* Regression: MSB must be in buf[8] -- any endian swap breaks the
     * event_id correlation key inference_daemon relies on to trace a
     * clip back to its originating PIR trigger. */
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
    cam_pack_header(buf, 1, 0xDEADBEEFCAFEBABEULL, 9800UL);
    cam_unpack_header(buf, &out);
    TEST_ASSERT_EQUAL_UINT32(CAM_MAGIC, out.magic);
    TEST_ASSERT_EQUAL_UINT8(CAM_HEADER_VERSION, out.version);
    TEST_ASSERT_EQUAL_UINT8(1, out.device_id);
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

void test_device_id_valid_at_max_slots_minus_one(void)
{
    TEST_ASSERT_EQUAL_INT(1, cam_device_id_valid(CAM_MAX_SLOTS - 1));
}

void test_device_id_invalid_at_max_slots(void)
{
    TEST_ASSERT_EQUAL_INT(0, cam_device_id_valid(CAM_MAX_SLOTS));
}

void test_device_id_invalid_255(void)
{
    TEST_ASSERT_EQUAL_INT(0, cam_device_id_valid(255));
}

/******************************************************************************
 * UDP trigger matching
 ******************************************************************************/

void test_trigger_parses_full_payload(void)
{
    cam_trigger_t trig;
    TEST_ASSERT_EQUAL_INT(1, cam_parse_trigger("CAPTURE:event_id=101,cam_tx_id=42,zone=0", &trig));
    TEST_ASSERT_EQUAL_UINT64(101, trig.event_id);
    TEST_ASSERT_EQUAL_UINT32(42,  trig.cam_tx_id);
    TEST_ASSERT_EQUAL_UINT8(0,    trig.zone);
}

void test_trigger_parses_nonzero_zone(void)
{
    cam_trigger_t trig;
    TEST_ASSERT_EQUAL_INT(1, cam_parse_trigger("CAPTURE:event_id=5,cam_tx_id=9,zone=2", &trig));
    TEST_ASSERT_EQUAL_UINT8(2, trig.zone);
}

void test_trigger_matches_prefix_only_fields_default_zero(void)
{
    cam_trigger_t trig;
    TEST_ASSERT_EQUAL_INT(1, cam_parse_trigger("CAPTURE:", &trig));
    TEST_ASSERT_EQUAL_UINT64(0, trig.event_id);
    TEST_ASSERT_EQUAL_UINT32(0, trig.cam_tx_id);
    TEST_ASSERT_EQUAL_UINT8(0,  trig.zone);
}

void test_trigger_no_match_empty(void)
{
    cam_trigger_t trig;
    TEST_ASSERT_EQUAL_INT(0, cam_parse_trigger("", &trig));
}

void test_trigger_no_match_wrong_prefix(void)
{
    cam_trigger_t trig;
    TEST_ASSERT_EQUAL_INT(0, cam_parse_trigger("TRIGGER:event_id=1", &trig));
}

void test_trigger_no_match_lowercase(void)
{
    cam_trigger_t trig;
    TEST_ASSERT_EQUAL_INT(0, cam_parse_trigger("capture:event_id=1", &trig));
}

void test_trigger_no_match_missing_colon(void)
{
    cam_trigger_t trig;
    TEST_ASSERT_EQUAL_INT(0, cam_parse_trigger("CAPTURE", &trig));
}

void test_trigger_null_buf_rejected(void)
{
    cam_trigger_t trig;
    TEST_ASSERT_EQUAL_INT(0, cam_parse_trigger(NULL, &trig));
}

void test_trigger_null_output_rejected(void)
{
    TEST_ASSERT_EQUAL_INT(0, cam_parse_trigger("CAPTURE:event_id=1,cam_tx_id=1,zone=1", NULL));
}

void test_trigger_prefix_value(void)
{
    TEST_ASSERT_EQUAL_STRING("CAPTURE:", CAM_TRIGGER_PREFIX);
}

/******************************************************************************
 * Network config constants -- BBB alignment regression guards
 ******************************************************************************/

void test_bbb_port(void)
{
    /* BBB TCP listener hardcoded to 9090 -- change breaks image receive */
    TEST_ASSERT_EQUAL_INT(9090, BBB_PORT);
}

void test_udp_trigger_port(void)
{
    /* Hub sends UDP triggers to 9091 -- must match hub CAM_TRIGGER_PORT */
    TEST_ASSERT_EQUAL_INT(9091, UDP_TRIGGER_PORT);
}

void test_reconnect_ms(void)
{
    TEST_ASSERT_EQUAL_INT(3000, RECONNECT_MS);
}

/******************************************************************************
 * Camera config constants
 ******************************************************************************/

void test_xclk_hz(void)
{
    /* 20MHz for OV2640 quality -- regression from 10MHz upscale failure */
    TEST_ASSERT_EQUAL_INT(20000000, CAM_XCLK_HZ);
}

void test_jpeg_quality(void)
{
    TEST_ASSERT_EQUAL_INT(12, CAM_JPEG_QUALITY);
}

void test_capture_retries(void)
{
    TEST_ASSERT_EQUAL_INT(5, CAM_CAPTURE_RETRIES);
}

/******************************************************************************
 * Camera pin constants -- silkscreen regression guards
 ******************************************************************************/

void test_cam_pin_xclk(void)  { TEST_ASSERT_EQUAL_INT(15, CAM_PIN_XCLK);  }
void test_cam_pin_siod(void)  { TEST_ASSERT_EQUAL_INT(4,  CAM_PIN_SIOD);  }
void test_cam_pin_sioc(void)  { TEST_ASSERT_EQUAL_INT(5,  CAM_PIN_SIOC);  }
void test_cam_pin_d7(void)    { TEST_ASSERT_EQUAL_INT(16, CAM_PIN_D7);    }
void test_cam_pin_d6(void)    { TEST_ASSERT_EQUAL_INT(17, CAM_PIN_D6);    }
void test_cam_pin_d5(void)    { TEST_ASSERT_EQUAL_INT(18, CAM_PIN_D5);    }
void test_cam_pin_d4(void)    { TEST_ASSERT_EQUAL_INT(12, CAM_PIN_D4);    }
void test_cam_pin_d3(void)    { TEST_ASSERT_EQUAL_INT(10, CAM_PIN_D3);    }
void test_cam_pin_d2(void)    { TEST_ASSERT_EQUAL_INT(8,  CAM_PIN_D2);    }
void test_cam_pin_d1(void)    { TEST_ASSERT_EQUAL_INT(9,  CAM_PIN_D1);    }
void test_cam_pin_d0(void)    { TEST_ASSERT_EQUAL_INT(11, CAM_PIN_D0);    }
void test_cam_pin_vsync(void) { TEST_ASSERT_EQUAL_INT(6,  CAM_PIN_VSYNC); }
void test_cam_pin_href(void)  { TEST_ASSERT_EQUAL_INT(7,  CAM_PIN_HREF);  }
void test_cam_pin_pclk(void)  { TEST_ASSERT_EQUAL_INT(13, CAM_PIN_PCLK);  }

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
    RUN_TEST(test_device_id_valid_at_max_slots_minus_one);
    RUN_TEST(test_device_id_invalid_at_max_slots);
    RUN_TEST(test_device_id_invalid_255);

    RUN_TEST(test_trigger_parses_full_payload);
    RUN_TEST(test_trigger_parses_nonzero_zone);
    RUN_TEST(test_trigger_matches_prefix_only_fields_default_zero);
    RUN_TEST(test_trigger_no_match_empty);
    RUN_TEST(test_trigger_no_match_wrong_prefix);
    RUN_TEST(test_trigger_no_match_lowercase);
    RUN_TEST(test_trigger_no_match_missing_colon);
    RUN_TEST(test_trigger_null_buf_rejected);
    RUN_TEST(test_trigger_null_output_rejected);
    RUN_TEST(test_trigger_prefix_value);

    RUN_TEST(test_bbb_port);
    RUN_TEST(test_udp_trigger_port);
    RUN_TEST(test_reconnect_ms);

    RUN_TEST(test_xclk_hz);
    RUN_TEST(test_jpeg_quality);
    RUN_TEST(test_capture_retries);

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
