/******************************************************************************
 * \file test_inference_main.c
 * \brief CUnit tests for inference_daemon pure logic.
 *
 * Coverage:
 *   infer_unpack_jpeg_len()  inference_core.h -- big-endian header parse
 *   infer_jpeg_len_valid()   inference_core.h -- bounds validation
 *   infer_above_threshold()  inference_core.h -- confidence gating
 *   infer_is_person()        inference_core.h -- label classification
 *   infer_label_for()        inference_core.h -- bounds-checked label lookup
 *   infer_detection_token()  inference_core.h -- filename token selection
 *   Config constants         inference_core.h -- port/threshold regression
 ******************************************************************************/

#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include <CUnit/Automated.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "inference_core.h"

/******************************************************************************
 * JPEG header unpacking
 ******************************************************************************/

void test_unpack_zero(void)
{
    uint8_t hdr[4] = {0, 0, 0, 0};
    CU_ASSERT_EQUAL(infer_unpack_jpeg_len(hdr), 0u);
}

void test_unpack_max(void)
{
    uint8_t hdr[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    CU_ASSERT_EQUAL(infer_unpack_jpeg_len(hdr), 0xFFFFFFFFu);
}

void test_unpack_known_value(void)
{
    uint8_t hdr[4] = {0x01, 0x02, 0x03, 0x04};
    CU_ASSERT_EQUAL(infer_unpack_jpeg_len(hdr), 0x01020304u);
}

void test_unpack_msb_first(void)
{
    /* Regression: MSB must be in hdr[0] -- any endian swap breaks framing */
    uint8_t hdr[4] = {0x00, 0x01, 0x00, 0x00};
    CU_ASSERT_EQUAL(infer_unpack_jpeg_len(hdr), 0x00010000u);
}

void test_unpack_roundtrip(void)
{
    /* Matches cam_pack_jpeg_header() on ESP32-CAM side */
    uint32_t len = 0xDEADBEEF;
    uint8_t hdr[4];
    hdr[0] = (len >> 24) & 0xFF;
    hdr[1] = (len >> 16) & 0xFF;
    hdr[2] = (len >>  8) & 0xFF;
    hdr[3] = (len      ) & 0xFF;
    CU_ASSERT_EQUAL(infer_unpack_jpeg_len(hdr), len);
}

void test_hdr_size_is_4(void)
{
    CU_ASSERT_EQUAL(JPEG_HDR_SIZE, 4);
}

/******************************************************************************
 * JPEG length validation
 ******************************************************************************/

void test_len_valid_typical(void)
{
    CU_ASSERT_EQUAL(infer_jpeg_len_valid(50000), 1);
}

void test_len_valid_min(void)
{
    CU_ASSERT_EQUAL(infer_jpeg_len_valid(1), 1);
}

void test_len_valid_at_max(void)
{
    CU_ASSERT_EQUAL(infer_jpeg_len_valid(MAX_JPEG_BYTES), 1);
}

void test_len_invalid_zero(void)
{
    CU_ASSERT_EQUAL(infer_jpeg_len_valid(0), 0);
}

void test_len_invalid_over_max(void)
{
    CU_ASSERT_EQUAL(infer_jpeg_len_valid(MAX_JPEG_BYTES + 1), 0);
}

void test_len_invalid_huge(void)
{
    CU_ASSERT_EQUAL(infer_jpeg_len_valid(0xFFFFFFFF), 0);
}

void test_max_jpeg_bytes_value(void)
{
    CU_ASSERT_EQUAL(MAX_JPEG_BYTES, 500000);
}

/******************************************************************************
 * Confidence threshold
 ******************************************************************************/

void test_threshold_above(void)
{
    CU_ASSERT_EQUAL(infer_above_threshold(0.9f), 1);
}

void test_threshold_at(void)
{
    CU_ASSERT_EQUAL(infer_above_threshold(0.5f), 1);
}

void test_threshold_below(void)
{
    CU_ASSERT_EQUAL(infer_above_threshold(0.49f), 0);
}

void test_threshold_zero(void)
{
    CU_ASSERT_EQUAL(infer_above_threshold(0.0f), 0);
}

void test_threshold_value(void)
{
    /* Regression: threshold change affects person/no-person boundary */
    CU_ASSERT_DOUBLE_EQUAL(CONFIDENCE_THRESH, 0.5f, 0.001f);
}

/******************************************************************************
 * Person label match
 ******************************************************************************/

void test_is_person_match(void)
{
    CU_ASSERT_EQUAL(infer_is_person("person"), 1);
}

void test_is_person_no_match_other(void)
{
    CU_ASSERT_EQUAL(infer_is_person("cat"), 0);
}

void test_is_person_no_match_empty(void)
{
    CU_ASSERT_EQUAL(infer_is_person(""), 0);
}

void test_is_person_no_match_null(void)
{
    CU_ASSERT_EQUAL(infer_is_person(NULL), 0);
}

void test_is_person_no_match_partial(void)
{
    CU_ASSERT_EQUAL(infer_is_person("pers"), 0);
}

void test_is_person_no_match_uppercase(void)
{
    CU_ASSERT_EQUAL(infer_is_person("Person"), 0);
}

/******************************************************************************
 * Label lookup
 ******************************************************************************/

void test_label_for_valid(void)
{
    char labels[MAX_LABELS][LABEL_LEN];
    strncpy(labels[0], "background", LABEL_LEN - 1);
    strncpy(labels[1], "person",     LABEL_LEN - 1);
    CU_ASSERT_STRING_EQUAL(infer_label_for(labels, 2, 1), "person");
}

void test_label_for_zero(void)
{
    char labels[MAX_LABELS][LABEL_LEN];
    strncpy(labels[0], "background", LABEL_LEN - 1);
    CU_ASSERT_STRING_EQUAL(infer_label_for(labels, 1, 0), "background");
}

void test_label_for_negative(void)
{
    char labels[MAX_LABELS][LABEL_LEN];
    CU_ASSERT_STRING_EQUAL(infer_label_for(labels, 1, -1), "");
}

void test_label_for_at_count(void)
{
    char labels[MAX_LABELS][LABEL_LEN];
    CU_ASSERT_STRING_EQUAL(infer_label_for(labels, 2, 2), "");
}

void test_label_for_empty_table(void)
{
    char labels[MAX_LABELS][LABEL_LEN];
    CU_ASSERT_STRING_EQUAL(infer_label_for(labels, 0, 0), "");
}

/******************************************************************************
 * Detection token
 ******************************************************************************/

void test_token_person(void)
{
    CU_ASSERT_STRING_EQUAL(infer_detection_token(1), "person");
}

void test_token_noperson(void)
{
    CU_ASSERT_STRING_EQUAL(infer_detection_token(0), "noperson");
}

/******************************************************************************
 * Config constants -- regression guards
 ******************************************************************************/

void test_listen_port(void)
{
    /* ESP32-CAM connects to this port -- must match BBB_PORT in cam_logic.h */
    CU_ASSERT_EQUAL(LISTEN_PORT, 9090);
}

void test_max_labels(void)
{
    CU_ASSERT_EQUAL(MAX_LABELS, 100);
}

void test_label_len(void)
{
    CU_ASSERT_EQUAL(LABEL_LEN, 64);
}

/******************************************************************************
 * Main
 ******************************************************************************/

int main(void)
{
    CU_initialize_registry();

    /* JPEG framing suite */
    CU_pSuite s_framing = CU_add_suite("jpeg_framing", NULL, NULL);
    CU_add_test(s_framing, "unpack_zero",       test_unpack_zero);
    CU_add_test(s_framing, "unpack_max",        test_unpack_max);
    CU_add_test(s_framing, "unpack_known",      test_unpack_known_value);
    CU_add_test(s_framing, "unpack_msb_first",  test_unpack_msb_first);
    CU_add_test(s_framing, "unpack_roundtrip",  test_unpack_roundtrip);
    CU_add_test(s_framing, "hdr_size_is_4",     test_hdr_size_is_4);

    /* Length validation suite */
    CU_pSuite s_len = CU_add_suite("jpeg_len_validation", NULL, NULL);
    CU_add_test(s_len, "valid_typical",       test_len_valid_typical);
    CU_add_test(s_len, "valid_min",           test_len_valid_min);
    CU_add_test(s_len, "valid_at_max",        test_len_valid_at_max);
    CU_add_test(s_len, "invalid_zero",        test_len_invalid_zero);
    CU_add_test(s_len, "invalid_over_max",    test_len_invalid_over_max);
    CU_add_test(s_len, "invalid_huge",        test_len_invalid_huge);
    CU_add_test(s_len, "max_jpeg_bytes",      test_max_jpeg_bytes_value);

    /* Confidence threshold suite */
    CU_pSuite s_thresh = CU_add_suite("confidence_threshold", NULL, NULL);
    CU_add_test(s_thresh, "above",           test_threshold_above);
    CU_add_test(s_thresh, "at",              test_threshold_at);
    CU_add_test(s_thresh, "below",           test_threshold_below);
    CU_add_test(s_thresh, "zero",            test_threshold_zero);
    CU_add_test(s_thresh, "value",           test_threshold_value);

    /* Person label suite */
    CU_pSuite s_label = CU_add_suite("person_label", NULL, NULL);
    CU_add_test(s_label, "match",            test_is_person_match);
    CU_add_test(s_label, "no_match_other",   test_is_person_no_match_other);
    CU_add_test(s_label, "no_match_empty",   test_is_person_no_match_empty);
    CU_add_test(s_label, "no_match_null",    test_is_person_no_match_null);
    CU_add_test(s_label, "no_match_partial", test_is_person_no_match_partial);
    CU_add_test(s_label, "no_match_upper",   test_is_person_no_match_uppercase);

    /* Label lookup suite */
    CU_pSuite s_lookup = CU_add_suite("label_lookup", NULL, NULL);
    CU_add_test(s_lookup, "valid",           test_label_for_valid);
    CU_add_test(s_lookup, "zero",            test_label_for_zero);
    CU_add_test(s_lookup, "negative",        test_label_for_negative);
    CU_add_test(s_lookup, "at_count",        test_label_for_at_count);
    CU_add_test(s_lookup, "empty_table",     test_label_for_empty_table);

    /* Detection token suite */
    CU_pSuite s_token = CU_add_suite("detection_token", NULL, NULL);
    CU_add_test(s_token, "person",           test_token_person);
    CU_add_test(s_token, "noperson",         test_token_noperson);

    /* Config constants suite */
    CU_pSuite s_cfg = CU_add_suite("config_constants", NULL, NULL);
    CU_add_test(s_cfg, "listen_port",        test_listen_port);
    CU_add_test(s_cfg, "max_labels",         test_max_labels);
    CU_add_test(s_cfg, "label_len",          test_label_len);

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();

#ifdef JUNIT_OUTPUT_PATH
    FILE *f = fopen(JUNIT_OUTPUT_PATH, "w");
    if (f) {
        int failures = CU_get_number_of_failures();
        int tests    = CU_get_number_of_tests_run();
        fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        fprintf(f, "<testsuite name=\"inference_tests\" tests=\"%d\" failures=\"%d\">\n",
                tests, failures);
        CU_pSuite s = CU_get_registry()->pSuite;
        while (s) {
            CU_pTest t = s->pTest;
            while (t) {
                fprintf(f, "  <testcase name=\"%s.%s\"/>\n", s->pName, t->pName);
                t = t->pNext;
            }
            s = s->pNext;
        }
        fprintf(f, "</testsuite>\n");
        fclose(f);
    }
#endif

    unsigned int failures = CU_get_number_of_failures();
    CU_cleanup_registry();
    return failures != 0;
}
