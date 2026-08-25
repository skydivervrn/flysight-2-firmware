// Host tests for the pure, bounded ActiveLook advertising parser.
#include <stdio.h>
#include <string.h>

#include "../FlySight/activelook_adv.h"

#define AD_UUID128_INCOMPLETE  0x06u
#define AD_UUID128_COMPLETE    0x07u
#define AD_NAME_SHORTENED      0x08u
#define AD_NAME_COMPLETE       0x09u

static const uint8_t kActiveLookUuid[16] = {
    0xB7, 0x5C, 0x49, 0xD2, 0x04, 0xA3, 0x40, 0x71,
    0xA0, 0xB5, 0x35, 0x85, 0x3E, 0xB0, 0x83, 0x07
};

static int g_fail;
static int g_checks;

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fail++; \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static void check_empty_result(const FS_ActiveLookAdvResult *r)
{
    CHECK(!r->found_uuid);
    CHECK(!r->found_name);
    CHECK(!r->have_serial);
    CHECK(r->name[0] == '\0');
    CHECK(r->serial[0] == '\0');
}

static void test_01_empty_report(void)
{
    FS_ActiveLookAdvResult r;
    CHECK(FS_ActiveLookAdv_Parse(NULL, 0, &r));
    check_empty_result(&r);
}

static void test_02_lone_length_byte(void)
{
    const uint8_t report[] = { 1u };
    FS_ActiveLookAdvResult r;
    CHECK(!FS_ActiveLookAdv_Parse(report, sizeof(report), &r));
    check_empty_result(&r);
}

static void test_03_adlength_exceeds_remaining(void)
{
    const uint8_t report[] = { 5u, AD_NAME_COMPLETE, 'E', 'N' };
    FS_ActiveLookAdvResult r;
    CHECK(!FS_ActiveLookAdv_Parse(report, sizeof(report), &r));
    check_empty_result(&r);
}

static void test_04_truncated_uuid(void)
{
    const uint8_t report[] = {
        9u, AD_UUID128_COMPLETE,
        0xB7, 0x5C, 0x49, 0xD2, 0x04, 0xA3, 0x40, 0x71
    };
    FS_ActiveLookAdvResult r;
    CHECK(FS_ActiveLookAdv_Parse(report, sizeof(report), &r));
    check_empty_result(&r);
}

static void test_05_multiple_valid_elements(void)
{
    uint8_t report[3u + 18u + 6u];
    size_t n = 0;
    FS_ActiveLookAdvResult r;

    report[n++] = 2u; report[n++] = 0x01u; report[n++] = 0x06u;
    report[n++] = 17u; report[n++] = AD_UUID128_INCOMPLETE;
    memcpy(&report[n], kActiveLookUuid, sizeof(kActiveLookUuid));
    n += sizeof(kActiveLookUuid);
    report[n++] = 5u; report[n++] = AD_NAME_SHORTENED;
    memcpy(&report[n], "al-x", 4u); n += 4u;

    CHECK(n == sizeof(report));
    CHECK(FS_ActiveLookAdv_Parse(report, n, &r));
    CHECK(r.found_uuid);
    CHECK(r.found_name);
    CHECK(!r.have_serial);
    CHECK(strcmp(r.name, "al-x") == 0);
    CHECK(r.serial[0] == '\0');
}

static void test_06_truncated_final_element_discards_report(void)
{
    const uint8_t report[] = {
        5u, AD_NAME_SHORTENED, 'E', 'N', 'G', 'O',
        7u, AD_NAME_COMPLETE, 'E', 'N'
    };
    FS_ActiveLookAdvResult r;
    CHECK(!FS_ActiveLookAdv_Parse(report, sizeof(report), &r));
    check_empty_result(&r);
}

static void test_07_complete_name_captures_serial(void)
{
    const uint8_t report[] = {
        14u, AD_NAME_COMPLETE,
        'E', 'N', 'G', 'O', ' ', '3', ' ', '1', '2', '3', '4', '5', '6'
    };
    FS_ActiveLookAdvResult r;
    CHECK(FS_ActiveLookAdv_Parse(report, sizeof(report), &r));
    CHECK(!r.found_uuid);
    CHECK(r.found_name);
    CHECK(r.have_serial);
    CHECK(strcmp(r.name, "ENGO 3 123456") == 0);
    CHECK(strcmp(r.serial, "123456") == 0);
}

static void test_08_shortened_name_never_captures_serial(void)
{
    const uint8_t report[] = {
        14u, AD_NAME_SHORTENED,
        'E', 'N', 'G', 'O', ' ', '3', ' ', '6', '5', '4', '3', '2', '1'
    };
    FS_ActiveLookAdvResult r;
    CHECK(FS_ActiveLookAdv_Parse(report, sizeof(report), &r));
    CHECK(!r.found_uuid);
    CHECK(r.found_name);
    CHECK(!r.have_serial);
    CHECK(strcmp(r.name, "ENGO 3 654321") == 0);
    CHECK(r.serial[0] == '\0');
}

/* A real report is padded with zeroes to the end of the payload, and the
 * padding is NOT malformed data (Core spec Vol 3, Part C, §11). A parser that
 * rejects the whole report on hitting it stops finding real glasses — the
 * exact failure the bounds checks exist to avoid causing. */
static void test_09_zero_padding_keeps_what_was_found(void)
{
    const uint8_t report[] = {
        14u, AD_NAME_COMPLETE,
        'E', 'N', 'G', 'O', ' ', '3', ' ', '0', '5', '0', '7', '1', '4',
        0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u
    };
    FS_ActiveLookAdvResult r;
    CHECK(FS_ActiveLookAdv_Parse(report, sizeof(report), &r));
    CHECK(r.found_name);
    CHECK(r.have_serial);
    CHECK(strcmp(r.serial, "050714") == 0);
}

/* Padding and nothing else: still a well-formed report, just an empty one. */
static void test_10_padding_only_is_not_a_failure(void)
{
    const uint8_t report[] = { 0u, 0u, 0u };
    FS_ActiveLookAdvResult r;
    CHECK(FS_ActiveLookAdv_Parse(report, sizeof(report), &r));
    check_empty_result(&r);
}

int main(void)
{
    test_01_empty_report();
    test_02_lone_length_byte();
    test_03_adlength_exceeds_remaining();
    test_04_truncated_uuid();
    test_05_multiple_valid_elements();
    test_06_truncated_final_element_discards_report();
    test_07_complete_name_captures_serial();
    test_08_shortened_name_never_captures_serial();
    test_09_zero_padding_keeps_what_was_found();
    test_10_padding_only_is_not_a_failure();

    printf("%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
