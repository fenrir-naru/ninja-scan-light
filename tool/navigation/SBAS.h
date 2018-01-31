/**
 * @file Satellite based augmentation system (SBAS)
 * @see DO-229D
 */

/*
 * Copyright (c) 2018, M.Naruoka (fenrir)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __SBAS_H__
#define __SBAS_H__

template <class FloatT = double>
class SBAS_SpaceNode {
  public:
    typedef FloatT float_t;

    struct RangingCode {
      int prn;
      int g2_delay_chips;
      int initial_g2;
      const char *name;
    };
    static const RangingCode ranging_codes[];

    enum MessageType {
      DONT_USE = 0,
      PRN_MASK = 1,
      FAST_CORRECTION_2 = 2,
      FAST_CORRECTION_3 = 3,
      FAST_CORRECTION_4 = 4,
      FAST_CORRECTION_5 = 5,
      INTEGRITY_INFORMATION = 6,
      FAST_CORRECTION_DEGRADATION = 7,
      GEO_NAVIGATION = 9,
      DEGRADATION_PARAMS = 10,
      SBAS_NETWORK_TIME_UTC_OFFSET_PARAMS = 12,
      GEO_SAT_ALNAMACS = 17,
      IONO_GRID_POINT_MASKS = 18,
      MIXED_CORRECTION_FAST_AND_LONG_TERM = 24,
      LONG_TERM_CORRECTION = 25,
      IONO_DELAY_CORRECTION = 26,
      SERVICE_MESSAGE = 27,
      CLOCK_EPHEMERIS_COV_MAT = 28,
      INTERNAL_TEST_MESSAGE = 62,
      NULL_MESSAGES = 63,
    }; ///< @see Table A-3
};

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::RangingCode SBAS_SpaceNode<FloatT>::ranging_codes[] = {
  {120,  145, 01106, "INMARSAT 3F2 AOR-E"},
  {121,  175, 01241, "INMARSAT 4F2"},
  {122,   52, 00267, "INMARSAT 3F4 AOR-W"},
  {123,   21, 00232, "LM RPS-1, RPS-2"},
  {124,  237, 01617, "Artemis"},
  {125,  235, 01076, "LM RPS-1, RPS-2"},
  {126,  886, 01764, "INMARSAT 3F5 IND-W"},
  {127,  657, 00717, "INSATNAV"},
  {128,  634, 01532, "INSATNAV"},
  {129,  762, 01250, "MTSAT-1R (or MTSAT-2)"},
  {130,  355, 00341, "INMARSAT 4F1"},
  {131, 1012, 00551, "INMARSAT 3F1 IOR"},
  {132,  176, 00520, "Unallocated"},
  {133,  603, 01731, "INMARSAT 4F3"},
  {134,  130, 00706, "INMARSAT 3F3 POR"},
  {135,  359, 01216, "LM RPS-1"},
  {136,  595, 00740, "INMARSAT Reserved"},
  {137,   68, 01007, "MTSAT-2 (or MTSAT-1R)"},
  {138,  386, 00450, "LM RPS-2"},
}; ///< @see Table A-1

#endif /* __SBAS_H__ */
