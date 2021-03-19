//
// HI-3593.h: interfaces a HOLT-IC dual RX, single TX ARINC 429 chip
//
// VERSION: V2.0 (added "float on ARINC" coding/decoding functions)
//
 
#ifndef HI3593_H
#define HI3593_H

// Shield initialization
void HI3593ShieldInit();

// sets the receiver #1 or #2 control register
void SetRxCtrlReg(
  unsigned char RxChannel, // ARINC 429 receive channel to configure
  unsigned char CtrlWord   // control word
);

// sets the label filter for a specified receiver channel
void SetLabelFilters(
  unsigned char RxChannel, // ARINC 429 receive channel to configure
  const unsigned char* bitarray  // bit array from label 377 down to label 000
);

// writes an ARINC word into the transmit FIFO
void WriteArincWord(unsigned long w);

// checks whether there is something in the receive FIFO of a prescribed receive channel
bool RxFifoEmpty(unsigned char RxChannel);

// reads an ARINC word from the receive FIFO of the prescribed receive channel
unsigned long ReadArincWord(unsigned char RxChannel);

// Builds a binary ARINC word from its separate constitutive fields
unsigned long BuildArincWord(
  const float &range,       // half range: if range is e.g. ±4096, set to 4096
  const float &data,        // the data to encode in the ARINC word
  const unsigned char ssm,  // the value to encode in SSM field (0..3)
  const unsigned char sdi,  // valid values are 0 to 3; if sdi>3 (e.g. = NOSDI), then uses SDI field as data
  const unsigned char label // the label to encode into the label field (bits flipped)
);

// Encodes a binary ARINC word from its separate constitutive fields (data transmitted is a 14-bit mantissa floating point)
unsigned long BuildArincWordFloat(
  const float &data,        // the data to encode in the ARINC word
  const bool vld,           // the validity (encode NaN into the data if invalid)
  const unsigned char label // the label to encode into the label field (bits flipped)
);

// Splits a binary ARINC word into Data, SSM and SDI
void SplitArincWord(
  const unsigned long &aw,  // the ARINC word to decode
  const bool noSdi,         // if true, the data is encoded on 21 bits, SDI field used
  const float &range,       // half range: if range is e.g. ±4096, set to 4096
  float &data,              // pointer to float where to store the decoded data
  unsigned char &ssm,       // pointer to the byte where to store the decoded SSM
  unsigned char &sdi        // pointer to the byte where to store the decoded SDI
);

// Splits a binary ARINC word into float data and validity
void SplitArincWordFloat(
  const unsigned long &aw,  // the ARINC word to decode
  float &data,              // pointer to float where to store the decoded data
  bool &vld                 // pointer to the byte where to store the decoded validity
);

// Combines two BNR SSMs (if both are NO => NO, otherwise if at least one is FW => FW, otherwise => NCD)
unsigned char cbnSsm(
  const unsigned char Ssm1,
  const unsigned char Ssm2
);

// constant to force usage of SDI field for data
#define NOSDI 4

// octal label decimal correspondences (with bits flipped)
#define o000 0
#define o001 128
#define o002 64
#define o003 192
#define o004 32
#define o005 160
#define o006 96
#define o007 224
#define o010 16
#define o011 144
#define o012 80
#define o013 208
#define o014 48
#define o015 176
#define o016 112
#define o017 240
#define o020 8
#define o021 136
#define o022 72
#define o023 200
#define o024 40
#define o025 168
#define o026 104
#define o027 232
#define o030 24
#define o031 152
#define o032 88
#define o033 216
#define o034 56
#define o035 184
#define o036 120
#define o037 248
#define o040 4
#define o041 132
#define o042 68
#define o043 196
#define o044 36
#define o045 164
#define o046 100
#define o047 228
#define o050 20
#define o051 148
#define o052 84
#define o053 212
#define o054 52
#define o055 180
#define o056 116
#define o057 244
#define o060 12
#define o061 140
#define o062 76
#define o063 204
#define o064 44
#define o065 172
#define o066 108
#define o067 236
#define o070 28
#define o071 156
#define o072 92
#define o073 220
#define o074 60
#define o075 188
#define o076 124
#define o077 252
#define o100 2
#define o101 130
#define o102 66
#define o103 194
#define o104 34
#define o105 162
#define o106 98
#define o107 226
#define o110 18
#define o111 146
#define o112 82
#define o113 210
#define o114 50
#define o115 178
#define o116 114
#define o117 242
#define o120 10
#define o121 138
#define o122 74
#define o123 202
#define o124 42
#define o125 170
#define o126 106
#define o127 234
#define o130 26
#define o131 154
#define o132 90
#define o133 218
#define o134 58
#define o135 186
#define o136 122
#define o137 250
#define o140 6
#define o141 134
#define o142 70
#define o143 198
#define o144 38
#define o145 166
#define o146 102
#define o147 230
#define o150 22
#define o151 150
#define o152 86
#define o153 214
#define o154 54
#define o155 182
#define o156 118
#define o157 246
#define o160 14
#define o161 142
#define o162 78
#define o163 206
#define o164 46
#define o165 174
#define o166 110
#define o167 238
#define o170 30
#define o171 158
#define o172 94
#define o173 222
#define o174 62
#define o175 190
#define o176 126
#define o177 254
#define o200 1
#define o201 129
#define o202 65
#define o203 193
#define o204 33
#define o205 161
#define o206 97
#define o207 225
#define o210 17
#define o211 145
#define o212 81
#define o213 209
#define o214 49
#define o215 177
#define o216 113
#define o217 241
#define o220 9
#define o221 137
#define o222 73
#define o223 201
#define o224 41
#define o225 169
#define o226 105
#define o227 233
#define o230 25
#define o231 153
#define o232 89
#define o233 217
#define o234 57
#define o235 185
#define o236 121
#define o237 249
#define o240 5
#define o241 133
#define o242 69
#define o243 197
#define o244 37
#define o245 165
#define o246 101
#define o247 229
#define o250 21
#define o251 149
#define o252 85
#define o253 213
#define o254 53
#define o255 181
#define o256 117
#define o257 245
#define o260 13
#define o261 141
#define o262 77
#define o263 205
#define o264 45
#define o265 173
#define o266 109
#define o267 237
#define o270 29
#define o271 157
#define o272 93
#define o273 221
#define o274 61
#define o275 189
#define o276 125
#define o277 253
#define o300 3
#define o301 131
#define o302 67
#define o303 195
#define o304 35
#define o305 163
#define o306 99
#define o307 227
#define o310 19
#define o311 147
#define o312 83
#define o313 211
#define o314 51
#define o315 179
#define o316 115
#define o317 243
#define o320 11
#define o321 139
#define o322 75
#define o323 203
#define o324 43
#define o325 171
#define o326 107
#define o327 235
#define o330 27
#define o331 155
#define o332 91
#define o333 219
#define o334 59
#define o335 187
#define o336 123
#define o337 251
#define o340 7
#define o341 135
#define o342 71
#define o343 199
#define o344 39
#define o345 167
#define o346 103
#define o347 231
#define o350 23
#define o351 151
#define o352 87
#define o353 215
#define o354 55
#define o355 183
#define o356 119
#define o357 247
#define o360 15
#define o361 143
#define o362 79
#define o363 207
#define o364 47
#define o365 175
#define o366 111
#define o367 239
#define o370 31
#define o371 159
#define o372 95
#define o373 223
#define o374 63
#define o375 191
#define o376 127
#define o377 255

#endif // HI3593_H
