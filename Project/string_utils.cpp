#include "string_utils.h"

std::vector<std::string> split(std::string str, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string temp;

    while (getline(ss, temp, delimiter)) {
        result.push_back(temp);
    }

    return result;
}

std::vector<double> double_parse(std::vector<std::string> split_line) {
    std::vector<double> tmp;
    tmp.reserve(5);
    tmp.push_back(std::stod(split_line[0]));
    tmp.push_back(std::stod(split_line[1]));
    tmp.push_back(std::stod(split_line[2]));
    tmp.push_back(std::stod(split_line[3]));
    tmp.push_back(std::stod(split_line[7]));

    return tmp;
}

size_t get_length(char* str) {
    size_t length = 0;
    const char* p = str;

    if (p == NULL) {
        return length;
    }

    while (*p != '\0') {
        ++length;
        p++;
    }

    return length;
}

int index_of(const char* str, const char* word) {
    const char* string_pointer = NULL;
    const char* word_pointer = NULL;
    int index = 0;

    if (get_length((char*)word) == 0) {
        return 0;
    }

    while (*str != '\0') {
        if (*str == *word) {
            string_pointer = str + 1;
            word_pointer = word + 1;
        }

        if (string_pointer != NULL && word_pointer != NULL) {
            while (*string_pointer != '\0' && *word_pointer != '\0' && *string_pointer == *word_pointer) {
                string_pointer++;
                word_pointer++;
            }

            if (*word_pointer == '\0') {
                return index;
            }

            if (*string_pointer == '\0') {
                break;
            }
        }

        str++;
        index++;
    }

    return -1;
}

char* tokenize(char* str_or_null, const char* delims) {
    static char* s_string_pointer = NULL;
    static int s_string_length = 0;
    static int s_count = 0;
    const char* delims_pointer = delims;
    int offset = 0;
    char* token = NULL;

    if (str_or_null != NULL) {
        s_string_pointer = str_or_null;
        s_string_length = get_length(s_string_pointer);
        s_count = 0;
    } else if (s_string_length == s_count) {
        return NULL;
    } else {
        s_string_pointer++;
    }

    while (*s_string_pointer != '\0') {
        while (*delims_pointer != '\0') {
            if (*s_string_pointer == *delims_pointer) {
                *s_string_pointer = '\0';
                s_count++;
                token = s_string_pointer - offset;

                if (*token != '\0') {
                    return token;
                }

                offset = -1;
            }

            delims_pointer++;
        }

        delims_pointer = delims;
        s_string_pointer++;
        s_count++;
        offset++;
    }

    token = s_string_pointer - offset;
    return token;
}
