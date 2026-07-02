/******************************************************************************
 * \file test_cam_main.c
 * \brief Unit tests for ESP32-S3-CAM node pure logic.
 *
 * Coverage:
 *   cam_pack_jpeg_header()   cam_logic.h  -- big-endian 4-byte length pack
 *   cam_unpack_jpeg_header() cam_logic.h  -- roundtrip correctness
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

void test_pack_zero(void)
{
    uint8_t hdr[4] = {0};
    cam_pack_jpeg_header(hdr, 0);
    TEST_ASSERT_EQUAL_UINT8(0, hdr[0]);
    TEST_ASSERT_EQUAL_UINT8(0, hdr[1]);
    TEST_ASSERT_EQUAL_UINT8(0, hdr[2]);
    TEST_ASSERT_EQUAL_UINT8(0, hdr[3]);
}

void test_pack_max(void)
{
    uint8_t hdr[4] = {0};
    cam_pack_jpeg_header(hdr, 0xFFFFFFFF);
    TEST_ASSERT_EQUAL_UINT8(0xFF, hdr[0]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, hdr[1]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, hdr[2]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, hdr[3]);
}

void test_pack_known_value(void)
{
    uint8_t hdr[4] = {0};
    cam_pack_jpeg_header(hdr, 0x01020304);
    TEST_ASSERT_EQUAL_UINT8(0x01, hdr[0]);
    TEST_ASSERT_EQUAL_UINT8(0x02, hdr[1]);
    TEST_ASSERT_EQUAL_UINT8(0x03, hdr[2]);
    TEST_ASSERT_EQUAL_UINT8(0x04, hdr[3]);
}

void test_pack_msb_first(void)
{
    /* Regression: BBB reads MSB first -- any endian swap breaks parser */
    uint8_t hdr[4] = {0};
    cam_pack_jpeg_header(hdr, 0x00000100);
    TEST_ASSERT_EQUAL_UINT8(0x00, hdr[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, hdr[1]);
    TEST_ASSERT_EQUAL_UINT8(0x01, hdr[2]);
    TEST_ASSERT_EQUAL_UINT8(0x00, hdr[3]);
}

void test_roundtrip_zero(void)
{
    uint8_t hdr[4] = {0};
    cam_pack_jpeg_header(hdr, 0);
    TEST_ASSERT_EQUAL_UINT32(0, cam_unpack_jpeg_header(hdr));
}

void test_roundtrip_max(void)
{
    uint8_t hdr[4] = {0};
    cam_pack_jpeg_header(hdr, 0xFFFFFFFF);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFF, cam_unpack_jpeg_header(hdr));
}

void test_roundtrip_known(void)
{
    uint8_t hdr[4] = {0};
    cam_pack_jpeg_header(hdr, 0xDEADBEEF);
    TEST_ASSERT_EQUAL_UINT32(0xDEADBEEF, cam_unpack_jpeg_header(hdr));
}

void test_hdr_size_is_4(void)
{
    TEST_ASSERT_EQUAL_INT(4, CAM_HDR_SIZE);
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

    RUN_TEST(test_pack_zero);
    RUN_TEST(test_pack_max);
    RUN_TEST(test_pack_known_value);
    RUN_TEST(test_pack_msb_first);
    RUN_TEST(test_roundtrip_zero);
    RUN_TEST(test_roundtrip_max);
    RUN_TEST(test_roundtrip_known);
    RUN_TEST(test_hdr_size_is_4);

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
