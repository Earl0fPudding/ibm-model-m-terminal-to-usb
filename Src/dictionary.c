#define DIC_SIZE 122

#include "usbhid.h"

typedef struct Mapping {
    uint8_t scancode;
    uint8_t hidcode;
} Mapping;

typedef struct Dictionary {
    Mapping maps[DIC_SIZE];
    uint8_t index;
} Dictionary;

uint8_t get_hidcode(Dictionary dictionary, uint8_t scancode) {
    for (uint8_t i = 0; i < dictionary.index + 1; ++i) {
        if (dictionary.maps[i].scancode == scancode) {
            return dictionary.maps[i].hidcode;
        }
    }
    return 0;
}

uint8_t scancode_is_in_dictionary(Dictionary dictionary, uint8_t scancode) {
    for (uint8_t i = 0; i < dictionary.index + 1; ++i) {
        if (dictionary.maps[i].scancode == scancode) {
            return 1;
        }
    }
    return 0;
}

void add_to_dic(Dictionary *dic, uint8_t scancode, uint8_t hidcode) {
    Mapping map;
    map.hidcode = hidcode;
    map.scancode = scancode;
    dic->maps[dic->index] = map;
    dic->index += 1;
}

void init_dictionary(Dictionary *dic) {
    dic->index = 0;
    // ADD KEYS
    // FUNCTION KEYS
    add_to_dic(dic, 0x8, HIDKEY_F13);
    add_to_dic(dic, 0x10, HIDKEY_F14);
    add_to_dic(dic, 0x18, HIDKEY_F15);
    add_to_dic(dic, 0x20, HIDKEY_F16);
    add_to_dic(dic, 0x28, HIDKEY_F17);
    add_to_dic(dic, 0x30, HIDKEY_F18);
    add_to_dic(dic, 0x38, HIDKEY_F19);
    add_to_dic(dic, 0x40, HIDKEY_F20);
    add_to_dic(dic, 0x48, HIDKEY_F21);
    add_to_dic(dic, 0x50, HIDKEY_F22);
    add_to_dic(dic, 0x57, HIDKEY_F23);
    add_to_dic(dic, 0x5f, HIDKEY_F24);
    add_to_dic(dic, 0x7, HIDKEY_F1);
    add_to_dic(dic, 0xf, HIDKEY_F2);
    add_to_dic(dic, 0x17, HIDKEY_F3);
    add_to_dic(dic, 0x1f, HIDKEY_F4);
    add_to_dic(dic, 0x27, HIDKEY_F5);
    add_to_dic(dic, 0x2f, HIDKEY_F6);
    add_to_dic(dic, 0x37, HIDKEY_F7);
    add_to_dic(dic, 0x3f, HIDKEY_F8);
    add_to_dic(dic, 0x47, HIDKEY_F9);
    add_to_dic(dic, 0x4f, HIDKEY_F10);
    add_to_dic(dic, 0x56, HIDKEY_F11);
    add_to_dic(dic, 0x5e, HIDKEY_F12);

    //UPPER NUMBER KEYS
    add_to_dic(dic, 0xe, HIDKEY_TILDE);
    add_to_dic(dic, 0x16, HIDKEY_1);
    add_to_dic(dic, 0x1e, HIDKEY_2);
    add_to_dic(dic, 0x26, HIDKEY_3);
    add_to_dic(dic, 0x25, HIDKEY_4);
    add_to_dic(dic, 0x2e, HIDKEY_5);
    add_to_dic(dic, 0x36, HIDKEY_6);
    add_to_dic(dic, 0x3d, HIDKEY_7);
    add_to_dic(dic, 0x3e, HIDKEY_8);
    add_to_dic(dic, 0x46, HIDKEY_9);
    add_to_dic(dic, 0x45, HIDKEY_0);
    add_to_dic(dic, 0x4e, HIDKEY_UNDERSCORE);
    add_to_dic(dic, 0x55, HIDKEY_PLUS);

    // ALPHABET CHARACTERS
    add_to_dic(dic, 0x15, HIDKEY_Q);
    add_to_dic(dic, 0x1d, HIDKEY_W);
    add_to_dic(dic, 0x24, HIDKEY_E);
    add_to_dic(dic, 0x2d, HIDKEY_R);
    add_to_dic(dic, 0x2c, HIDKEY_T);
    add_to_dic(dic, 0x35, HIDKEY_Y);
    add_to_dic(dic, 0x3c, HIDKEY_U);
    add_to_dic(dic, 0x43, HIDKEY_I);
    add_to_dic(dic, 0x44, HIDKEY_O);
    add_to_dic(dic, 0x4d, HIDKEY_P);
    add_to_dic(dic, 0x1c, HIDKEY_A);
    add_to_dic(dic, 0x1b, HIDKEY_S);
    add_to_dic(dic, 0x23, HIDKEY_D);
    add_to_dic(dic, 0x2b, HIDKEY_F);
    add_to_dic(dic, 0x34, HIDKEY_G);
    add_to_dic(dic, 0x33, HIDKEY_H);
    add_to_dic(dic, 0x3b, HIDKEY_J);
    add_to_dic(dic, 0x42, HIDKEY_K);
    add_to_dic(dic, 0x4b, HIDKEY_L);
    add_to_dic(dic, 0x1a, HIDKEY_Z);
    add_to_dic(dic, 0x22, HIDKEY_X);
    add_to_dic(dic, 0x21, HIDKEY_C);
    add_to_dic(dic, 0x2a, HIDKEY_V);
    add_to_dic(dic, 0x32, HIDKEY_B);
    add_to_dic(dic, 0x31, HIDKEY_N);
    add_to_dic(dic, 0x3a, HIDKEY_M);

    //ALPHABET SPECIAL
    add_to_dic(dic, 0x54, HIDKEY_OPEN_BRACKET);
    add_to_dic(dic, 0x5b, HIDKEY_CLOSE_BRACKET);
    add_to_dic(dic, 0x4c, HIDKEY_COLON);
    add_to_dic(dic, 0x52, HIDKEY_QUOTE);
    add_to_dic(dic, 0x53, HIDKEY_HASH);
    add_to_dic(dic, 0x41, HIDKEY_COMMA);
    add_to_dic(dic, 0x49, HIDKEY_DOT);
    add_to_dic(dic, 0x4a, HIDKEY_SLASH);
    add_to_dic(dic, 0x13, 0x64);

    //SPECIAL KEYS
    add_to_dic(dic, 0xd, HIDKEY_TAB);
    add_to_dic(dic, 0x14, HIDKEY_CAPS_LOCK);
    add_to_dic(dic, 0x12, HIDKEY_MODIFIER_LEFT_SHIFT);
    add_to_dic(dic, 0x11, HIDKEY_MODIFIER_LEFT_CTRL);
    add_to_dic(dic, 0x19, HIDKEY_MODIFIER_LEFT_ALT);
    add_to_dic(dic, 0x29, HIDKEY_SPACEBAR);
    add_to_dic(dic, 0x39, HIDKEY_MODIFIER_RIGHT_ALT);
    add_to_dic(dic, 0x58, HIDKEY_MODIFIER_RIGHT_CTRL);
    add_to_dic(dic, 0x66, HIDKEY_BACKSPACE);
    add_to_dic(dic, 0x5a, HIDKEY_ENTER);
    add_to_dic(dic, 0x59, HIDKEY_MODIFIER_RIGHT_SHIFT);

    // ARROW KEYS
    add_to_dic(dic, 0x63, HIDKEY_UP);
    add_to_dic(dic, 0x60, HIDKEY_DOWN);
    add_to_dic(dic, 0x61, HIDKEY_LEFT);
    add_to_dic(dic, 0x6a, HIDKEY_RIGHT);

    // NUMPAD
    add_to_dic(dic, 0x70, HIDKEY_KP_0);
    add_to_dic(dic, 0x69, HIDKEY_KP_1);
    add_to_dic(dic, 0x72, HIDKEY_KP_2);
    add_to_dic(dic, 0x7a, HIDKEY_KP_3);
    add_to_dic(dic, 0x6b, HIDKEY_KP_4);
    add_to_dic(dic, 0x73, HIDKEY_KP_5);
    add_to_dic(dic, 0x74, HIDKEY_KP_6);
    add_to_dic(dic, 0x6c, HIDKEY_KP_7);
    add_to_dic(dic, 0x75, HIDKEY_KP_8);
    add_to_dic(dic, 0x7d, HIDKEY_KP_9);
    add_to_dic(dic, 0x71, HIDKEY_COMMA);
    add_to_dic(dic, 0x7e, HIDKEY_DOT);
    add_to_dic(dic, 0x79, HIDKEY_KP_ENTER);
    add_to_dic(dic, 0x7b, HIDKEY_KP_PLUS);
    add_to_dic(dic, 0x7c, HIDKEY_KP_MINUS);
    add_to_dic(dic, 0x76, HIDKEY_KP_NUM_LOCK);
}