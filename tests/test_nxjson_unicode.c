// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

/* Boundary and invalid-scalar regressions for nxjson's UTF-8 encoder. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../third_party/openhmd/src/ext_deps/nxjson.c"

static int check(unsigned int cp, const unsigned char *expected, size_t len)
{
    unsigned char bytes[8];
    memset(bytes, 0xa5, sizeof bytes);
    char *end = NULL;
    int ok = unicode_to_utf8(cp, (char *)bytes, &end);
    if (!ok || end != (char *)bytes + len || memcmp(bytes, expected, len) || bytes[len] != 0xa5) {
        fprintf(stderr, "UTF-8 mismatch at U+%04X\n", cp);
        return 1;
    }
    return 0;
}

int main(void)
{
    int errors = 0;
    errors += check(0, (const unsigned char *)"\x00", 1);
    errors += check(0x7f, (const unsigned char *)"\x7f", 1);
    errors += check(0x80, (const unsigned char *)"\xc2\x80", 2);
    errors += check(0x7ff, (const unsigned char *)"\xdf\xbf", 2);
    errors += check(0x800, (const unsigned char *)"\xe0\xa0\x80", 3);
    errors += check(0xd7ff, (const unsigned char *)"\xed\x9f\xbf", 3);
    errors += check(0xe000, (const unsigned char *)"\xee\x80\x80", 3);
    errors += check(0xffff, (const unsigned char *)"\xef\xbf\xbf", 3);
    errors += check(0x10000, (const unsigned char *)"\xf0\x90\x80\x80", 4);
    errors += check(0x10ffff, (const unsigned char *)"\xf4\x8f\xbf\xbf", 4);
    const unsigned int invalid[] = {0xd800, 0xdbff, 0xdc00, 0xdfff, 0x110000, UINT32_MAX};
    for (size_t i = 0; i < sizeof invalid / sizeof invalid[0]; ++i) {
        unsigned char bytes[8];
        memset(bytes, 0xa5, sizeof bytes);
        char *end = (char *)bytes + 7;
        if (unicode_to_utf8(invalid[i], (char *)bytes, &end) || end != (char *)bytes + 7 || bytes[0] != 0xa5)
            ++errors;
    }
    char text[] = "\"\\ud800\\udc00\"";
    const nx_json *json = nx_json_parse_utf8(text);
    if (!json || strcmp(json->text_value, "\xf0\x90\x80\x80")) ++errors;
    nx_json_free(json);
    return errors != 0;
}
