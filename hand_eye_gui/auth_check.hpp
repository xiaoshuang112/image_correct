#ifndef AUTH_CHECK_HPP
#define AUTH_CHECK_HPP

#include <string>
#include <vector>
#include <array>
#include <memory>
#include <cstring>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <algorithm>

// ===================================================================
//  密钥常量（来自 det-xialiao-oak 加密体系）
// ===================================================================
const unsigned char HEADER_LEN = 32;

static unsigned char HEADER_KEY[32] = {
    0xf1,0x8a,0x75,0xf5, 0xce,0xfd,0x49,0xe4, 0x89,0xfb,0x2b,0xd3,0x7e,0x18,0x40,0x9a,
    0x95,0xbc,0x82,0xfb, 0xeb,0x69,0x40,0xa0, 0x9b,0xd4,0x8e,0x8b,0x94,0x83,0x26,0x8d
};

static unsigned char HEADERMERGE[32] = {
    0xef,0xc9,0x4b,0xdf, 0x2e,0x6b,0x45,0x07, 0x87,0x5d,0xae,0xf7,0xce,0x4c,0xce,0xed,
    0x77,0xfd,0xf9,0xc1, 0x24,0xa6,0x40,0xc4, 0x81,0x9b,0x3e,0x06,0x2c,0x48,0x01,0xf9
};

static unsigned char PADDING[] = {
    0x80,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

static char g_md5[33] = { 0 };

// ===================================================================
//  简易字符串工具（替代 alva_str_util）
// ===================================================================
namespace str_util {
    inline void replace(std::string& target, const std::string& search,
                        const std::string& replacement) {
        if (search == replacement || search.empty()) return;
        std::string::size_type lastPos = 0;
        while ((lastPos = target.find(search, lastPos)) != std::string::npos) {
            target.replace(lastPos, search.length(), replacement);
            lastPos += replacement.length();
        }
    }
    inline std::string upperCase(std::string value) {
        for (auto& c : value) c = static_cast<char>(std::toupper(c));
        return value;
    }
    inline std::string hextostr(const char* hexval, int hex_len) {
        std::stringstream sstr;
        sstr.setf(std::ios_base::hex, std::ios::basefield);
        sstr.fill('0');
        for (int i = 0; i < hex_len; i++)
            sstr.width(2), sstr << (unsigned int)(unsigned char)hexval[i];
        return sstr.str();
    }
}

// ===================================================================
//  MD5 实现
// ===================================================================
typedef struct {
    unsigned int count[2];
    unsigned int state[4];
    unsigned char buffer[64];
} MD5_CTX;

#define F(x,y,z) ((x & y) | (~x & z))
#define G(x,y,z) ((x & z) | (y & ~z))
#define H(x,y,z) (x^y^z)
#define I(x,y,z) (y ^ (x | ~z))
#define ROTATE_LEFT(x,n) ((x << n) | (x >> (32-n)))
#define FF(a,b,c,d,x,s,ac) { a+=F(b,c,d)+x+ac; a=ROTATE_LEFT(a,s); a+=b; }
#define GG(a,b,c,d,x,s,ac) { a+=G(b,c,d)+x+ac; a=ROTATE_LEFT(a,s); a+=b; }
#define HH(a,b,c,d,x,s,ac) { a+=H(b,c,d)+x+ac; a=ROTATE_LEFT(a,s); a+=b; }
#define II(a,b,c,d,x,s,ac) { a+=I(b,c,d)+x+ac; a=ROTATE_LEFT(a,s); a+=b; }

inline void MD5Encode(unsigned char* output, unsigned int* input, unsigned int len) {
    unsigned int i = 0, j = 0;
    while (j < len) {
        output[j] = input[i] & 0xFF;
        output[j+1] = (input[i] >> 8) & 0xFF;
        output[j+2] = (input[i] >> 16) & 0xFF;
        output[j+3] = (input[i] >> 24) & 0xFF;
        i++; j += 4;
    }
}

inline void MD5Decode(unsigned int* output, unsigned char* input, unsigned int len) {
    unsigned int i = 0, j = 0;
    while (j < len) {
        output[i] = (input[j]) | (input[j+1] << 8) |
                    (input[j+2] << 16) | (input[j+3] << 24);
        i++; j += 4;
    }
}

inline void MD5Transform(unsigned int state[4], unsigned char block[64]) {
    unsigned int a = state[0], b = state[1], c = state[2], d = state[3], x[64];
    MD5Decode(x, block, 64);
    FF(a,b,c,d,x[0],7,0xd76aa478); FF(d,a,b,c,x[1],12,0xe8c7b756);
    FF(c,d,a,b,x[2],17,0x242070db); FF(b,c,d,a,x[3],22,0xc1bdceee);
    FF(a,b,c,d,x[4],7,0xf57c0faf); FF(d,a,b,c,x[5],12,0x4787c62a);
    FF(c,d,a,b,x[6],17,0xa8304613); FF(b,c,d,a,x[7],22,0xfd469501);
    FF(a,b,c,d,x[8],7,0x698098d8); FF(d,a,b,c,x[9],12,0x8b44f7af);
    FF(c,d,a,b,x[10],17,0xffff5bb1); FF(b,c,d,a,x[11],22,0x895cd7be);
    FF(a,b,c,d,x[12],7,0x6b901122); FF(d,a,b,c,x[13],12,0xfd987193);
    FF(c,d,a,b,x[14],17,0xa679438e); FF(b,c,d,a,x[15],22,0x49b40821);
    GG(a,b,c,d,x[1],5,0xf61e2562); GG(d,a,b,c,x[6],9,0xc040b340);
    GG(c,d,a,b,x[11],14,0x265e5a51); GG(b,c,d,a,x[0],20,0xe9b6c7aa);
    GG(a,b,c,d,x[5],5,0xd62f105d); GG(d,a,b,c,x[10],9,0x2441453);
    GG(c,d,a,b,x[15],14,0xd8a1e681); GG(b,c,d,a,x[4],20,0xe7d3fbc8);
    GG(a,b,c,d,x[9],5,0x21e1cde6); GG(d,a,b,c,x[14],9,0xc33707d6);
    GG(c,d,a,b,x[3],14,0xf4d50d87); GG(b,c,d,a,x[8],20,0x455a14ed);
    GG(a,b,c,d,x[13],5,0xa9e3e905); GG(d,a,b,c,x[2],9,0xfcefa3f8);
    GG(c,d,a,b,x[7],14,0x676f02d9); GG(b,c,d,a,x[12],20,0x8d2a4c8a);
    HH(a,b,c,d,x[5],4,0xfffa3942); HH(d,a,b,c,x[8],11,0x8771f681);
    HH(c,d,a,b,x[11],16,0x6d9d6122); HH(b,c,d,a,x[14],23,0xfde5380c);
    HH(a,b,c,d,x[1],4,0xa4beea44); HH(d,a,b,c,x[4],11,0x4bdecfa9);
    HH(c,d,a,b,x[7],16,0xf6bb4b60); HH(b,c,d,a,x[10],23,0xbebfbc70);
    HH(a,b,c,d,x[13],4,0x289b7ec6); HH(d,a,b,c,x[0],11,0xeaa127fa);
    HH(c,d,a,b,x[3],16,0xd4ef3085); HH(b,c,d,a,x[6],23,0x4881d05);
    HH(a,b,c,d,x[9],4,0xd9d4d039); HH(d,a,b,c,x[12],11,0xe6db99e5);
    HH(c,d,a,b,x[15],16,0x1fa27cf8); HH(b,c,d,a,x[2],23,0xc4ac5665);
    II(a,b,c,d,x[0],6,0xf4292244); II(d,a,b,c,x[7],10,0x432aff97);
    II(c,d,a,b,x[14],15,0xab9423a7); II(b,c,d,a,x[5],21,0xfc93a039);
    II(a,b,c,d,x[12],6,0x655b59c3); II(d,a,b,c,x[3],10,0x8f0ccc92);
    II(c,d,a,b,x[10],15,0xffeff47d); II(b,c,d,a,x[1],21,0x85845dd1);
    II(a,b,c,d,x[8],6,0x6fa87e4f); II(d,a,b,c,x[15],10,0xfe2ce6e0);
    II(c,d,a,b,x[6],15,0xa3014314); II(b,c,d,a,x[13],21,0x4e0811a1);
    II(a,b,c,d,x[4],6,0xf7537e82); II(d,a,b,c,x[11],10,0xbd3af235);
    II(c,d,a,b,x[2],15,0x2ad7d2bb); II(b,c,d,a,x[9],21,0xeb86d391);
    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
}

inline void MD5Init(MD5_CTX* ctx) {
    ctx->count[0] = 0; ctx->count[1] = 0;
    ctx->state[0] = 0x67452301; ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE; ctx->state[3] = 0x10325476;
}

inline void MD5Update(MD5_CTX* ctx, unsigned char* input, unsigned int len) {
    unsigned int i = 0, index = (ctx->count[0] >> 3) & 0x3F;
    unsigned int partlen = 64 - index;
    ctx->count[0] += len << 3;
    if (ctx->count[0] < (len << 3)) ctx->count[1]++;
    ctx->count[1] += len >> 29;
    if (len >= partlen) {
        memcpy(&ctx->buffer[index], input, partlen);
        MD5Transform(ctx->state, ctx->buffer);
        for (i = partlen; i + 64 <= len; i += 64)
            MD5Transform(ctx->state, &input[i]);
        index = 0;
    } else i = 0;
    memcpy(&ctx->buffer[index], &input[i], len - i);
}

inline void MD5Final(MD5_CTX* ctx, unsigned char digest[16]) {
    unsigned int index = (ctx->count[0] >> 3) & 0x3F;
    unsigned int padlen = (index < 56) ? (56 - index) : (120 - index);
    unsigned char bits[8];
    MD5Encode(bits, ctx->count, 8);
    MD5Update(ctx, PADDING, padlen);
    MD5Update(ctx, bits, 8);
    MD5Encode(digest, ctx->state, 16);
}

inline char* GetMD5(char* src_str) {
    std::string dst_mac(src_str);
    if (dst_mac.empty()) { g_md5[0] = 0; return g_md5; }

    str_util::replace(dst_mac, ":", "");
    str_util::replace(dst_mac, "-", "");
    dst_mac = str_util::upperCase(dst_mac);

    int str_len = (int)dst_mac.length();
    char* dst_str = new char[64];
    memset(dst_str, 0, 64);
    memcpy(dst_str, HEADER_KEY, 32);

    for (int i = 0; i < str_len && i < 64; i++) {
        dst_str[i] = dst_mac[i] ^ HEADER_KEY[i];
        if (dst_str[i] == 0) dst_str[i] = 48;
    }

    char enc_str[16] = { 0 };
    MD5_CTX md5;
    MD5Init(&md5);
    MD5Update(&md5, (unsigned char*)dst_str, str_len);
    MD5Final(&md5, (unsigned char*)enc_str);

    std::string dst_md5 = str_util::hextostr(enc_str, 16);
    for (int i = 0; i < 32; i++) g_md5[i] = dst_md5[i];
    g_md5[32] = 0;
    delete[] dst_str;
    return g_md5;
}

// ===================================================================
//  MAC 地址获取
// ===================================================================
inline std::vector<std::string> execute_command(const std::string& cmd) {
    std::vector<std::string> output;
    std::array<char, 128> buffer;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) return output;
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        output.emplace_back(buffer.data());
    return output;
}

inline std::vector<std::string> get_mac_address_list() {
    std::vector<std::string> list_mac;
    auto output = execute_command("ifconfig ");
    for (const auto& line : output) {
        if (line.find("ether") != std::string::npos) {
            std::istringstream iss(line);
            std::string token;
            while (iss >> token) {
                if (token == "ether") {
                    iss >> token;
                    list_mac.emplace_back(token);
                    break;
                }
            }
        }
    }
    return list_mac;
}

// ===================================================================
//  解密函数
// ===================================================================
inline int decode_buf(const char* merge_buf, unsigned long long merge_len,
                      std::string mac, std::string encode_key,
                      std::vector<char>& data_buf) {
    if (nullptr == merge_buf || merge_len < sizeof(unsigned long long) + HEADER_LEN)
        return -1;

    const char* p = merge_buf;
    unsigned long long data_len = *(unsigned long long*)p;
    p += sizeof(unsigned long long);

    if (data_len < HEADER_LEN ||
        HEADER_LEN + (data_len - HEADER_LEN) > merge_len - sizeof(unsigned long long))
        return -1;

    char sn[HEADER_LEN + 1] = { 0 };
    memcpy(sn, p, HEADER_LEN);
    sn[HEADER_LEN] = '\0';
    p += HEADER_LEN;

    std::string mac_key = mac + encode_key;
    std::vector<char> key_buf(mac_key.begin(), mac_key.end());
    key_buf.push_back('\0');
    char* expected_sn = GetMD5(key_buf.data());
    if (expected_sn[0] == 0) return -1;

    for (int i = 0; i < HEADER_LEN; i++) {
        if (sn[i] != expected_sn[i]) return -2;
    }

    unsigned long long content_len = data_len - HEADER_LEN;
    unsigned char combined_key[HEADER_LEN];
    for (int i = 0; i < HEADER_LEN; i++)
        combined_key[i] = HEADERMERGE[i] ^ (unsigned char)sn[i];

    data_buf.clear();
    data_buf.resize(content_len);
    for (unsigned long long i = 0; i < content_len; i++)
        data_buf[i] = ((unsigned char)p[i] - combined_key[i % HEADER_LEN]) & 0xFF;

    return 0;
}

// ===================================================================
//  授权检查（由 main.cpp 调用）
//  返回空字符串 = 授权通过，否则返回错误消息
// ===================================================================
std::string check_authorization(const std::string& appPath);

#endif // AUTH_CHECK_HPP
