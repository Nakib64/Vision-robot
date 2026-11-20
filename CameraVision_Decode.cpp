#include "CameraVision.h"

// ---------------------------------------------------------------------------
// ArUco DICT_4X4_50 Dictionary (50 markers, 16 bits each, flash memory)
// ---------------------------------------------------------------------------
static const uint16_t ARUCO_DICT_4X4_50[50] = {
    0xB532, 0x0F9A, 0x332D, 0x9946, 0x549E,
    0x79CD, 0x9E2E, 0xC4F2, 0xFEDA, 0xCF56,
    0xF991, 0x11A7, 0x0EB7, 0x2A0F, 0x24B1,
    0x263E, 0x4665, 0x6600, 0x6C5E, 0x76AF,
    0x868B, 0xB02B, 0xCCD5, 0xDD82, 0xFE47,
    0x9471, 0xACE4, 0xA554, 0x2123, 0x346F,
    0x4415, 0x57B2, 0x9ECF, 0xF0CB, 0x08AE,
    0x0929, 0x1875, 0x04FF, 0x0DF6, 0x1C5A,
    0x1718, 0x2A28, 0x328C, 0x38B2, 0x24E8,
    0x2EEB, 0x2D3F, 0x4B64, 0x502E, 0x5013
};

// ---------------------------------------------------------------------------
// AprilTag tag16h5 Dictionary (30 markers, 16 bits each, flash memory)
// ---------------------------------------------------------------------------
static const uint16_t APRILTAG_DICT_16H5[30] = {
    0xD8C4, 0xA574, 0x562C, 0x9DA2, 0x659E,
    0xD6FE, 0x1ACD, 0xA2E7, 0x9A7F, 0xB6A8,
    0xD01C, 0xD50F, 0x21B0, 0x6CE2, 0x4E31,
    0x08F5, 0x3C90, 0x2DC9, 0xC0A5, 0xF162,
    0xEC87, 0xA9EA, 0x42FB, 0xB838, 0x3B97,
    0xB5CE, 0xFAB5, 0x0CAB, 0x53E0, 0x74F5
};

// ---------------------------------------------------------------------------
// decodeAruco4x4() — Mirror & Rotation Invariant 4x4 Dictionary Matcher
// ---------------------------------------------------------------------------
int CameraVision::decodeAruco4x4(const uint8_t inner[4][4], int &outHamming) {
    int bestId   = -1;
    int bestDist = 99;

    // Test both normal and horizontally mirrored orientations
    for (int flip = 0; flip < 2; flip++) {
        for (int rot = 0; rot < 4; rot++) {
            uint16_t code = 0;
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    int col = flip ? (3 - c) : c;
                    uint8_t bit = 0;
                    if (rot == 0)      bit = inner[r][col];
                    else if (rot == 1) bit = inner[3 - col][r];
                    else if (rot == 2) bit = inner[3 - r][3 - col];
                    else if (rot == 3) bit = inner[col][3 - r];
                    code = (code << 1) | (bit & 1);
                }
            }
            // Check ArUco DICT_4X4_50 (50 markers) FIRST (primary target dictionary)
            for (int id = 0; id < 50; id++) {
                int dist = __builtin_popcount(code ^ ARUCO_DICT_4X4_50[id]);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestId   = id;
                }
            }
            // Check AprilTag tag16h5 (30 markers) as secondary fallback only if no clean ArUco match
            if (bestDist > 1) {
                for (int id = 0; id < 30; id++) {
                    int dist = __builtin_popcount(code ^ APRILTAG_DICT_16H5[id]);
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestId   = id;
                    }
                }
            }
        }
    }
    outHamming = bestDist;
    return bestId;
}
