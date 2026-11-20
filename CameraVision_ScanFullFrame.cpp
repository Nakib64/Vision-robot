#include "CameraVision.h"
#include <math.h>

static unsigned long s_lastDiagLog = 0;

// ---------------------------------------------------------------------------
// MODE A: Full-Frame ArUco Discovery with Dynamic Lighting & Nearest Selection
// ---------------------------------------------------------------------------
TargetDetection CameraVision::scanFullFrame(camera_fb_t *fb) {
    const int W        = (int)fb->width;   // 320
    const int H        = (int)fb->height;  // 240
    const int rowLimit = H - CROP_BOTTOM;  // 215

    // 1. Measure frame lighting (mean & min) — immune to bright ceiling lights (255)
    uint8_t minVal = 255;
    uint8_t maxVal = 0;
    long    sumVal = 0;
    int     nSamp  = 0;
    for (int y = 20; y < rowLimit; y += 20) {
        for (int x = 20; x < W - 20; x += 28) {
            uint8_t v = fb->buf[y * W + x];
            if (v < minVal) minVal = v;
            if (v > maxVal) maxVal = v;
            sumVal += v;
            nSamp++;
        }
    }
    uint8_t meanVal = (nSamp > 0) ? (uint8_t)(sumVal / nSamp) : 100;

    // Three adaptive thresholds for all lighting environments (dim, normal, bright):
    uint8_t passThresholds[3] = {
        (uint8_t)constrain((int)(minVal + (meanVal - minVal) * 0.45f), 35, 80),
        (uint8_t)constrain((int)(minVal + (meanVal - minVal) * 0.65f), 45, 95),
        (uint8_t)constrain((int)(minVal + (meanVal - minVal) * 0.85f), 65, 125)
    };

    // Component tracking for connected dark regions
    struct CandidateBlob {
        int  minX, maxX, minY, maxY;
        int  count;
        long sumX, sumY;
    };

    // Declared static to avoid stack allocation on FreeRTOS stack
    static constexpr int MAX_RUNS  = 256;
    static constexpr int MAX_COMPS = 128;

    struct RowRun { int startX, endX, compIdx; };

    static CandidateBlob comps[MAX_COMPS];
    static int           compParent[MAX_COMPS];
    static RowRun        prevRuns[MAX_RUNS];
    static RowRun        curRuns[MAX_RUNS];

    TargetDetection bestTarget = emptyResult();
    int lastNumComps = 0;
    int lastTested   = 0;
    int lastValid    = 0;
    uint8_t usedThresh = passThresholds[1];

    for (int pass = 0; pass < 3 && !bestTarget.found; pass++) {
        uint8_t darkThresh = passThresholds[pass];
        usedThresh = darkThresh;

        int numComps     = 0;
        int prevRunCount = 0;

        auto findRoot = [&](int i) -> int {
            while (compParent[i] != i) {
                compParent[i] = compParent[compParent[i]];
                i = compParent[i];
            }
            return i;
        };

        auto unionComps = [&](int i, int j) {
            int ri = findRoot(i);
            int rj = findRoot(j);
            if (ri != rj) {
                compParent[rj] = ri;
                comps[ri].minX  = min(comps[ri].minX, comps[rj].minX);
                comps[ri].maxX  = max(comps[ri].maxX, comps[rj].maxX);
                comps[ri].minY  = min(comps[ri].minY, comps[rj].minY);
                comps[ri].maxY  = max(comps[ri].maxY, comps[rj].maxY);
                comps[ri].count += comps[rj].count;
                comps[ri].sumX  += comps[rj].sumX;
                comps[ri].sumY  += comps[rj].sumY;
            }
        };

        // Scan rows with step of 2 for maximum speed (~2 ms)
        for (int y = 0; y < rowLimit; y += 2) {
            const uint8_t *row = fb->buf + y * W;
            int curRunCount = 0;
            bool inRun = false;
            int sx = 0;

            for (int x = 0; x < W; x++) {
                if (row[x] <= darkThresh) {
                    if (!inRun) {
                        inRun = true;
                        sx = x;
                    }
                } else {
                    if (inRun) {
                        inRun = false;
                        if (curRunCount < MAX_RUNS) {
                            curRuns[curRunCount++] = {sx, x - 1, -1};
                        }
                    }
                }
            }
            if (inRun && curRunCount < MAX_RUNS) {
                curRuns[curRunCount++] = {sx, W - 1, -1};
            }

            // Link with overlapping runs from previous scanned row
            for (int i = 0; i < curRunCount; i++) {
                int myComp = -1;
                for (int p = 0; p < prevRunCount; p++) {
                    if (prevRuns[p].compIdx < 0) continue;
                    if (!(curRuns[i].endX < prevRuns[p].startX || curRuns[i].startX > prevRuns[p].endX)) {
                        if (myComp == -1) {
                            myComp = findRoot(prevRuns[p].compIdx);
                        } else {
                            unionComps(myComp, prevRuns[p].compIdx);
                            myComp = findRoot(myComp);
                        }
                    }
                }

                int cnt = curRuns[i].endX - curRuns[i].startX + 1;
                long smX = (long)(curRuns[i].startX + curRuns[i].endX) * cnt / 2;
                long smY = (long)y * cnt;

                if (myComp == -1) {
                    if (numComps < MAX_COMPS) {
                        myComp = numComps++;
                        compParent[myComp] = myComp;
                        comps[myComp] = {curRuns[i].startX, curRuns[i].endX, y, y, cnt, smX, smY};
                    }
                } else {
                    comps[myComp].minX  = min(comps[myComp].minX, curRuns[i].startX);
                    comps[myComp].maxX  = max(comps[myComp].maxX, curRuns[i].endX);
                    comps[myComp].maxY  = max(comps[myComp].maxY, y);
                    comps[myComp].count += cnt;
                    comps[myComp].sumX  += smX;
                    comps[myComp].sumY  += smY;
                }
                curRuns[i].compIdx = myComp;
            }

            prevRunCount = min(curRunCount, MAX_RUNS);
            for (int i = 0; i < prevRunCount; i++) {
                prevRuns[i] = curRuns[i];
            }
        }

        // Evaluate root components as ArUco candidates
        float bestDist = 9999.0f;
        int testedCandidates = 0;
        int validCount = 0;

        for (int i = 0; i < numComps; i++) {
            if (compParent[i] != i) continue; // Root components only

            CandidateBlob &b = comps[i];
            int bw = b.maxX - b.minX + 1;
            int bh = b.maxY - b.minY + 1;

            if (bw < MIN_MARKER_PX || bh < MIN_MARKER_PX || bw > MAX_MARKER_PX || bh > MAX_MARKER_PX) {
                continue;
            }

            // Aspect ratio: ArUco markers are square (allows tilt / perspective foreshortening)
            float aspect = (float)bh / (float)bw;
            if (aspect < 0.45f || aspect > 2.20f) continue;

            testedCandidates++;

            // 1. Measure border darkness and find brightest inner cell directly from marker
            long borderSum   = 0;
            int  borderTotal = 0;
            uint8_t innerMin = 255;
            uint8_t innerMax = 0;

            for (int r = 0; r < 6; r++) {
                for (int c = 0; c < 6; c++) {
                    int sx = constrain(b.minX + (int)((c + 0.5f) * bw / 6.0f), 0, W - 1);
                    int sy = constrain(b.minY + (int)((r + 0.5f) * bh / 6.0f), 0, rowLimit - 1);
                    uint8_t val = fb->buf[sy * W + sx];

                    if (r == 0 || r == 5 || c == 0 || c == 5) {
                        borderSum += val;
                        borderTotal++;
                    } else {
                        if (val < innerMin) innerMin = val;
                        if (val > innerMax) innerMax = val;
                    }
                }
            }

            uint8_t borderAvg = (borderTotal > 0) ? (uint8_t)(borderSum / borderTotal) : 50;

            // Must have contrast between black border and inner white cells (at least 8 levels)
            // Completely independent of external quiet zone (works on dark chairs, dark walls, stacked markers)
            if (innerMax <= borderAvg + 8) {
                continue; // Solid black blob or no white cells
            }

            // Optimal adaptive local threshold between marker's own black and white cells
            uint8_t localThresh = (uint8_t)(((int)borderAvg + (int)innerMax) / 2);

            // 2. Verify border is primarily dark
            int borderDarkCount = 0;
            for (int r = 0; r < 6; r++) {
                for (int c = 0; c < 6; c++) {
                    if (r == 0 || r == 5 || c == 0 || c == 5) {
                        int sx = constrain(b.minX + (int)((c + 0.5f) * bw / 6.0f), 0, W - 1);
                        int sy = constrain(b.minY + (int)((r + 0.5f) * bh / 6.0f), 0, rowLimit - 1);
                        if (fb->buf[sy * W + sx] <= localThresh) {
                            borderDarkCount++;
                        }
                    }
                }
            }

            if (borderTotal > 0 && ((float)borderDarkCount / (float)borderTotal) < 0.30f) {
                continue; // Border is not dark enough
            }

            // 3. Sample inner 4x4 data bits using local threshold
            uint8_t inner[4][4];
            int onesCount = 0;
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    int sx = constrain(b.minX + (int)((c + 1.5f) * bw / 6.0f), 0, W - 1);
                    int sy = constrain(b.minY + (int)((r + 1.5f) * bh / 6.0f), 0, rowLimit - 1);
                    inner[r][c] = (fb->buf[sy * W + sx] > localThresh) ? 1 : 0;
                    if (inner[r][c]) onesCount++;
                }
            }

            // A valid pattern must have a mix of cells (at least 2 ones and at most 14 ones)
            if (onesCount < 2 || onesCount > 14) {
                continue;
            }

            // 4. Decode against ArUco DICT_4X4_50 with up to 2-bit error correction
            int hamming = 99;
            int decodedId = decodeAruco4x4(inner, hamming);
            bool validMatch = (decodedId >= 0 && hamming <= 2);

#if defined(TARGET_ARUCO_ID) && (TARGET_ARUCO_ID >= 0)
            if (decodedId != TARGET_ARUCO_ID) {
                validMatch = false;
            }
#endif

            // Check Visited Target Memory (6-second exclusion cooldown)
            if (validMatch && isVisited(decodedId)) {
                validMatch = false;
            }

            if (validMatch) {
                int candCenterX = (b.count > 0) ? (int)(b.sumX / b.count) : (b.minX + bw / 2);
                float apparentWidth = (float)max(bw, bh);
                float distCm = (TARGET_REAL_WIDTH_CM * CAMERA_FX_PX) / max(apparentWidth, 1.0f);

                validCount++;

                // Multi-detection nearest selection: keep closest marker
                if (!bestTarget.found || distCm < bestDist) {
                    bestDist = distCm;
                    bestTarget.found           = true;
                    bestTarget.tagId           = decodedId;
                    bestTarget.centerX         = candCenterX;
                    bestTarget.centerY         = (b.count > 0) ? (int)(b.sumY / b.count) : (b.minY + bh / 2);
                    bestTarget.bboxX           = b.minX;
                    bestTarget.bboxY           = b.minY;
                    bestTarget.bboxW           = bw;
                    bestTarget.bboxH           = bh;
                    bestTarget.apparentWidthPx = apparentWidth;
                    bestTarget.estimatedDistCm = distCm;
                    bestTarget.error           = bestTarget.centerX - FRAME_CENTER_X;
                    bestTarget.angleDeg        = atan2f((float)bestTarget.error, CAMERA_FX_PX) * 180.0f / (float)M_PI;

                    // True optical angle relative to 90° center Y-axis
                    bestTarget.polarAngleDeg   = constrain(90.0f - bestTarget.angleDeg, 0.0f, 180.0f);
                }
            }
        }
        lastNumComps = numComps;
        lastTested   = testedCandidates;
        lastValid    = validCount;
    } // end multi-threshold pass loop

    // Periodic telemetry log every 1.5 seconds if searching
    unsigned long now = millis();
    if (!bestTarget.found && (now - s_lastDiagLog > 1500)) {
        s_lastDiagLog = now;
        Serial.printf("[VISION:DIAG] Min:%d Mean:%d Thresh:%d/%d | Comps:%d Cand:%d\n",
                      minVal, meanVal, passThresholds[0], passThresholds[1], lastNumComps, lastTested);
    }

    if (bestTarget.found) {
        Serial.printf("[CAM:DISCOVER] Nearest (1/%d) ID:%d | Dist:%.1f cm | Ang:%+.1f deg | Th:%d\n",
                      lastValid, bestTarget.tagId, bestTarget.estimatedDistCm,
                      bestTarget.angleDeg, usedThresh);
    }

    return bestTarget;
}
