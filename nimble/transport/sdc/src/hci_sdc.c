/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <stdlib.h>

/* BLE */
#include <nimble/transport/hci_h4.h>

#include "nimble/ble.h"
#include "nimble/hci_common.h"
#include "nimble/nimble_npl.h"
#include "nimble/nimble_opt.h"
#include "nimble/transport.h"
#include "sdc/mpsl.h"
#include "sdc/sdc.h"
#include "sdc/sdc_hci.h"
#include "sdc/sdc_hci_cmd_controller_baseband.h"
#include "sdc/sdc_hci_cmd_info_params.h"
#include "sdc/sdc_hci_cmd_le.h"
#include "sdc/sdc_hci_cmd_link_control.h"
#include "sdc/sdc_hci_cmd_status_params.h"
#include "sdc/sdc_soc.h"

static const uint8_t test_6_pattern[] = {
    /* Random data */
    0x00, 0x02, 0x00, 0x04, 0x00, 0x06, 0x00, 0x08, 0x00, 0x0a, 0x00, 0x0c, 0x00, 0x0e, 0x00, 0x10,
    0x00, 0x12, 0x00, 0x14, 0x00, 0x16, 0x00, 0x18, 0x00, 0x1a, 0x00, 0x1c, 0x00, 0x1e, 0x00, 0x20,
    0x00, 0x22, 0x00, 0x24, 0x00, 0x26, 0x00, 0x28, 0x00, 0x2a, 0x00, 0x2c, 0x00, 0x2e, 0x00, 0x30,
    0x00, 0x32, 0x00, 0x34, 0x00, 0x36, 0x00, 0x38, 0x00, 0x3a, 0x00, 0x3c, 0x00, 0x3e, 0x00, 0x40,
    0x00, 0x42, 0x00, 0x44, 0x00, 0x46, 0x00, 0x48, 0x00, 0x4a, 0x00, 0x4c, 0x00, 0x4e, 0x00, 0x50,
    0x00, 0x52, 0x00, 0x54, 0x00, 0x56, 0x00, 0x58, 0x00, 0x5a, 0x00, 0x5c, 0x00, 0x5e, 0x00, 0x60,
    0x00, 0x62, 0x00, 0x64, 0x00, 0x66, 0x00, 0x68, 0x00, 0x6a, 0x00, 0x6c, 0x00, 0x6e, 0x00, 0x70,
    0x00, 0x72, 0x00, 0x74, 0x00, 0x76, 0x00, 0x78, 0x00, 0x7a, 0x00, 0x7c, 0x00, 0x7e, 0x00, 0x80,
    0x00, 0x82, 0x00, 0x84, 0x00, 0x86, 0x00, 0x88, 0x00, 0x8a, 0x00, 0x8c, 0x00, 0x8e, 0x00, 0x90,
    0x00, 0x92, 0x00, 0x94, 0x00, 0x96, 0x00, 0x98, 0x00, 0x9a, 0x00, 0x9c, 0x00, 0x9e, 0x00, 0xa0,
    0x00, 0xa2, 0x00, 0xa4, 0x00, 0xa6, 0x00, 0xa8, 0x00, 0xaa, 0x00, 0xac, 0x00, 0xae, 0x00, 0xb0,
    0x00, 0xb2, 0x00, 0xb4, 0x00, 0xb6, 0x00, 0xb8, 0x00, 0xba, 0x00, 0xbc, 0x00, 0xbe, 0x00, 0xc0,
    0x00, 0xc2, 0x00, 0xc4, 0x00, 0xc6, 0x00, 0xc8, 0x00, 0xca, 0x00, 0xcc, 0x00, 0xce, 0x00, 0xd0,
    0x00, 0xd2, 0x00, 0xd4, 0x00, 0xd6, 0x00, 0xd8, 0x00, 0xda, 0x00, 0xdc, 0x00, 0xde, 0x00, 0xe0,
    0x00, 0xe2, 0x00, 0xe4, 0x00, 0xe6, 0x00, 0xe8, 0x00, 0xea, 0x00, 0xec, 0x00, 0xee, 0x00, 0xf0,
    0x00, 0xf2, 0x00, 0xf4, 0x00, 0xf6, 0x00, 0xf8, 0x00, 0xfa, 0x00, 0xfc, 0x00, 0xfe, 0x01, 0x01,
    0x01, 0x03, 0x01, 0x05, 0x01, 0x07, 0x01, 0x09, 0x01, 0x0b, 0x01, 0x0d, 0x01, 0x0f, 0x01, 0x11,
    0x01, 0x13, 0x01, 0x15, 0x01, 0x17, 0x01, 0x19, 0x01, 0x1b, 0x01, 0x1d, 0x01, 0x1f, 0x01, 0x21,
    0x01, 0x23, 0x01, 0x25, 0x01, 0x27, 0x01, 0x29, 0x01, 0x2b, 0x01, 0x2d, 0x01, 0x2f, 0x01, 0x31,
    0x01, 0x33, 0x01, 0x35, 0x01, 0x37, 0x01, 0x39, 0x01, 0x3b, 0x01, 0x3d, 0x01, 0x3f, 0x01, 0x41,
    0x01, 0x43, 0x01, 0x45, 0x01, 0x47, 0x01, 0x49, 0x01, 0x4b, 0x01, 0x4d, 0x01, 0x4f, 0x01, 0x51,
    0x01, 0x53, 0x01, 0x55, 0x01, 0x57, 0x01, 0x59, 0x01, 0x5b, 0x01, 0x5d, 0x01, 0x5f, 0x01, 0x61,
    0x01, 0x63, 0x01, 0x65, 0x01, 0x67, 0x01, 0x69, 0x01, 0x6b, 0x01, 0x6d, 0x01, 0x6f, 0x01, 0x71,
    0x01, 0x73, 0x01, 0x75, 0x01, 0x77, 0x01, 0x79, 0x01, 0x7b, 0x01, 0x7d, 0x01, 0x7f, 0x01, 0x81,
    0x01, 0x83, 0x01, 0x85, 0x01, 0x87, 0x01, 0x89, 0x01, 0x8b, 0x01, 0x8d, 0x01, 0x8f, 0x01, 0x91,
    0x01, 0x93, 0x01, 0x95, 0x01, 0x97, 0x01, 0x99, 0x01, 0x9b, 0x01, 0x9d, 0x01, 0x9f, 0x01, 0xa1,
    0x01, 0xa3, 0x01, 0xa5, 0x01, 0xa7, 0x01, 0xa9, 0x01, 0xab, 0x01, 0xad, 0x01, 0xaf, 0x01, 0xb1,
    0x01, 0xb3, 0x01, 0xb5, 0x01, 0xb7, 0x01, 0xb9, 0x01, 0xbb, 0x01, 0xbd, 0x01, 0xbf, 0x01, 0xc1,
    0x01, 0xc3, 0x01, 0xc5, 0x01, 0xc7, 0x01, 0xc9, 0x01, 0xcb, 0x01, 0xcd, 0x01, 0xcf, 0x01, 0xd1,
    0x01, 0xd3, 0x01, 0xd5, 0x01, 0xd7, 0x01, 0xd9, 0x01, 0xdb, 0x01, 0xdd, 0x01, 0xdf, 0x01, 0xe1,
    0x01, 0xe3, 0x01, 0xe5, 0x01, 0xe7, 0x01, 0xe9, 0x01, 0xeb, 0x01, 0xed, 0x01, 0xef, 0x01, 0xf1,
    0x01, 0xf3, 0x01, 0xf5, 0x01, 0xf7, 0x01, 0xf9, 0x01, 0xfb, 0x01, 0xfd, 0x02, 0x00, 0x02, 0x02,
    0x02, 0x04, 0x02, 0x06, 0x02, 0x08, 0x02, 0x0a, 0x02, 0x0c, 0x02, 0x0e, 0x02, 0x10, 0x02, 0x12,
    0x02, 0x14, 0x02, 0x16, 0x02, 0x18, 0x02, 0x1a, 0x02, 0x1c, 0x02, 0x1e, 0x02, 0x20, 0x02, 0x22,
    0x02, 0x24, 0x02, 0x26, 0x02, 0x28, 0x02, 0x2a, 0x02, 0x2c, 0x02, 0x2e, 0x02, 0x30, 0x02, 0x32,
    0x02, 0x34, 0x02, 0x36, 0x02, 0x38, 0x02, 0x3a, 0x02, 0x3c, 0x02, 0x3e, 0x02, 0x40, 0x02, 0x42,
    0x02, 0x44, 0x02, 0x46, 0x02, 0x48, 0x02, 0x4a, 0x02, 0x4c, 0x02, 0x4e, 0x02, 0x50, 0x02, 0x52,
    0x02, 0x54, 0x02, 0x56, 0x02, 0x58, 0x02, 0x5a, 0x02, 0x5c, 0x02, 0x5e, 0x02, 0x60, 0x02, 0x62,
    0x02, 0x64, 0x02, 0x66, 0x02, 0x68, 0x02, 0x6a, 0x02, 0x6c, 0x02, 0x6e, 0x02, 0x70, 0x02, 0x72,
    0x02, 0x74, 0x02, 0x76, 0x02, 0x78, 0x02, 0x7a, 0x02, 0x7c, 0x02, 0x7e, 0x02, 0x80, 0x02, 0x82,
    0x02, 0x84, 0x02, 0x86, 0x02, 0x88, 0x02, 0x8a, 0x02, 0x8c, 0x02, 0x8e, 0x02, 0x90, 0x02, 0x92,
    0x02, 0x94, 0x02, 0x96, 0x02, 0x98, 0x02, 0x9a, 0x02, 0x9c, 0x02, 0x9e, 0x02, 0xa0, 0x02, 0xa2,
    0x02, 0xa4, 0x02, 0xa6, 0x02, 0xa8, 0x02, 0xaa, 0x02, 0xac, 0x02, 0xae, 0x02, 0xb0, 0x02, 0xb2,
    0x02, 0xb4, 0x02, 0xb6, 0x02, 0xb8, 0x02, 0xba, 0x02, 0xbc, 0x02, 0xbe, 0x02, 0xc0, 0x02, 0xc2,
    0x02, 0xc4, 0x02, 0xc6, 0x02, 0xc8, 0x02, 0xca, 0x02, 0xcc, 0x02, 0xce, 0x02, 0xd0, 0x02, 0xd2,
    0x02, 0xd4, 0x02, 0xd6, 0x02, 0xd8, 0x02, 0xda, 0x02, 0xdc, 0x02, 0xde, 0x02, 0xe0, 0x02, 0xe2,
    0x02, 0xe4, 0x02, 0xe6, 0x02, 0xe8, 0x02, 0xea, 0x02, 0xec, 0x02, 0xee, 0x02, 0xf0, 0x02, 0xf2,
    0x02, 0xf4, 0x02, 0xf6, 0x02, 0xf8, 0x02, 0xfa, 0x02, 0xfc, 0x02, 0xfe, 0x03, 0x01, 0x03, 0x03,
    0x03, 0x05, 0x03, 0x07, 0x03, 0x09, 0x03, 0x0b, 0x03, 0x0d, 0x03, 0x0f, 0x03, 0x11, 0x03, 0x13,
    0x03, 0x15, 0x03, 0x17, 0x03, 0x19, 0x03, 0x1b, 0x03, 0x1d, 0x03, 0x1f, 0x03, 0x21, 0x03, 0x23,
    0x03, 0x25, 0x03, 0x27, 0x03, 0x29, 0x03, 0x2b, 0x03, 0x2d, 0x03, 0x2f, 0x03, 0x31, 0x03, 0x33,
    0x03, 0x35, 0x03, 0x37, 0x03, 0x39, 0x03, 0x3b, 0x03, 0x3d, 0x03, 0x3f, 0x03, 0x41, 0x03, 0x43,
    0x03, 0x45, 0x03, 0x47, 0x03, 0x49, 0x03, 0x4b, 0x03, 0x4d, 0x03, 0x4f, 0x03, 0x51, 0x03, 0x53,
    0x03, 0x55, 0x03, 0x57, 0x03, 0x59, 0x03, 0x5b, 0x03, 0x5d, 0x03, 0x5f, 0x03, 0x61, 0x03, 0x63,
    0x03, 0x65, 0x03, 0x67, 0x03, 0x69, 0x03, 0x6b, 0x03, 0x6d, 0x03, 0x6f, 0x03, 0x71, 0x03, 0x73,
    0x03, 0x75, 0x03, 0x77, 0x03, 0x79, 0x03, 0x7b, 0x03, 0x7d, 0x03, 0x7f, 0x03, 0x81, 0x03, 0x83,
    0x03, 0x85, 0x03, 0x87, 0x03, 0x89, 0x03, 0x8b, 0x03, 0x8d, 0x03, 0x8f, 0x03, 0x91, 0x03, 0x93,
    0x03, 0x95, 0x03, 0x97, 0x03, 0x99, 0x03, 0x9b, 0x03, 0x9d, 0x03, 0x9f, 0x03, 0xa1, 0x03, 0xa3,
    0x03, 0xa5, 0x03, 0xa7, 0x03, 0xa9, 0x03, 0xab, 0x03, 0xad, 0x03, 0xaf, 0x03, 0xb1, 0x03, 0xb3,
    0x03, 0xb5, 0x03, 0xb7, 0x03, 0xb9, 0x03, 0xbb, 0x03, 0xbd, 0x03, 0xbf, 0x03, 0xc1, 0x03, 0xc3,
    0x03, 0xc5, 0x03, 0xc7, 0x03, 0xc9, 0x03, 0xcb, 0x03, 0xcd, 0x03, 0xcf, 0x03, 0xd1, 0x03, 0xd3,
    0x03, 0xd5, 0x03, 0xd7, 0x03, 0xd9, 0x03, 0xdb, 0x03, 0xdd, 0x03, 0xdf, 0x03, 0xe1, 0x03, 0xe3,
    0x03, 0xe5, 0x03, 0xe7, 0x03, 0xe9, 0x03, 0xeb, 0x03, 0xed, 0x03, 0xef, 0x03, 0xf1, 0x03, 0xf3,
    0x03, 0xf5, 0x03, 0xf7, 0x03, 0xf9, 0x03, 0xfb, 0x03, 0xfd, 0x04, 0x00, 0x04, 0x02, 0x04, 0x04,
    0x04, 0x06, 0x04, 0x08, 0x04, 0x0a, 0x04, 0x0c, 0x04, 0x0e, 0x04, 0x10, 0x04, 0x12, 0x04, 0x14,
    0x04, 0x16, 0x04, 0x18, 0x04, 0x1a, 0x04, 0x1c, 0x04, 0x1e, 0x04, 0x20, 0x04, 0x22, 0x04, 0x24,
    0x04, 0x26, 0x04, 0x28, 0x04, 0x2a, 0x04, 0x2c, 0x04, 0x2e, 0x04, 0x30, 0x04, 0x32, 0x04, 0x34,
    0x04, 0x36, 0x04, 0x38, 0x04, 0x3a, 0x04, 0x3c, 0x04, 0x3e, 0x04, 0x40, 0x04, 0x42, 0x04, 0x44,
    0x04, 0x46, 0x04, 0x48, 0x04, 0x4a, 0x04, 0x4c, 0x04, 0x4e, 0x04, 0x50, 0x04, 0x52, 0x04, 0x54,
    0x04, 0x56, 0x04, 0x58, 0x04, 0x5a, 0x04, 0x5c, 0x04, 0x5e, 0x04, 0x60, 0x04, 0x62, 0x04, 0x64,
    0x04, 0x66, 0x04, 0x68, 0x04, 0x6a, 0x04, 0x6c, 0x04, 0x6e, 0x04, 0x70, 0x04, 0x72, 0x04, 0x74,
    0x04, 0x76, 0x04, 0x78, 0x04, 0x7a, 0x04, 0x7c, 0x04, 0x7e, 0x04, 0x80, 0x04, 0x82, 0x04, 0x84,
    0x04, 0x86, 0x04, 0x88, 0x04, 0x8a, 0x04, 0x8c, 0x04, 0x8e, 0x04, 0x90, 0x04, 0x92, 0x04, 0x94,
    0x04, 0x96, 0x04, 0x98, 0x04, 0x9a, 0x04, 0x9c, 0x04, 0x9e, 0x04, 0xa0, 0x04, 0xa2, 0x04, 0xa4,
    0x04, 0xa6, 0x04, 0xa8, 0x04, 0xaa, 0x04, 0xac, 0x04, 0xae, 0x04, 0xb0, 0x04, 0xb2, 0x04, 0xb4,
    0x04, 0xb6, 0x04, 0xb8, 0x04, 0xba, 0x04, 0xbc, 0x04, 0xbe, 0x04, 0xc0, 0x04, 0xc2, 0x04, 0xc4,
    0x04, 0xc6, 0x04, 0xc8, 0x04, 0xca, 0x04, 0xcc, 0x04, 0xce, 0x04, 0xd0, 0x04, 0xd2, 0x04, 0xd4,
    0x04, 0xd6, 0x04, 0xd8, 0x04, 0xda, 0x04, 0xdc, 0x04, 0xde, 0x04, 0xe0, 0x04, 0xe2, 0x04, 0xe4,
    0x04, 0xe6, 0x04, 0xe8, 0x04, 0xea, 0x04, 0xec, 0x04, 0xee, 0x04, 0xf0, 0x04, 0xf2, 0x04, 0xf4,
    0x04, 0xf6, 0x04, 0xf8, 0x04, 0xfa, 0x04, 0xfc, 0x04, 0xfe, 0x05, 0x01, 0x05, 0x03, 0x05, 0x05,
    0x05, 0x07, 0x05, 0x09, 0x05, 0x0b, 0x05, 0x0d, 0x05, 0x0f, 0x05, 0x11, 0x05, 0x13, 0x05, 0x15,
    0x05, 0x17, 0x05, 0x19, 0x05, 0x1b, 0x05, 0x1d, 0x05, 0x1f, 0x05, 0x21, 0x05, 0x23, 0x05, 0x25,
    0x05, 0x27, 0x05, 0x29, 0x05, 0x2b, 0x05, 0x2d, 0x05, 0x2f, 0x05, 0x31, 0x05, 0x33, 0x05, 0x35,
    0x05, 0x37, 0x05, 0x39, 0x05, 0x3b, 0x05, 0x3d, 0x05, 0x3f, 0x05, 0x41, 0x05, 0x43, 0x05, 0x45,
    0x05, 0x47, 0x05, 0x49, 0x05, 0x4b, 0x05, 0x4d, 0x05, 0x4f, 0x05, 0x51, 0x05, 0x53, 0x05, 0x55,
    0x05, 0x57, 0x05, 0x59, 0x05, 0x5b, 0x05, 0x5d, 0x05, 0x5f, 0x05, 0x61, 0x05, 0x63, 0x05, 0x65,
    0x05, 0x67, 0x05, 0x69, 0x05, 0x6b, 0x05, 0x6d, 0x05, 0x6f, 0x05, 0x71, 0x05, 0x73, 0x05, 0x75,
    0x05, 0x77, 0x05, 0x79, 0x05, 0x7b, 0x05, 0x7d, 0x05, 0x7f, 0x05, 0x81, 0x05, 0x83, 0x05, 0x85,
    0x05, 0x87, 0x05, 0x89, 0x05, 0x8b, 0x05, 0x8d, 0x05, 0x8f, 0x05, 0x91, 0x05, 0x93, 0x05, 0x95,
    0x05, 0x97, 0x05, 0x99, 0x05, 0x9b, 0x05, 0x9d, 0x05, 0x9f, 0x05, 0xa1, 0x05, 0xa3, 0x05, 0xa5,
    0x05, 0xa7, 0x05, 0xa9, 0x05, 0xab, 0x05, 0xad, 0x05, 0xaf, 0x05, 0xb1, 0x05, 0xb3, 0x05, 0xb5,
    0x05, 0xb7, 0x05, 0xb9, 0x05, 0xbb, 0x05, 0xbd, 0x05, 0xbf, 0x05, 0xc1, 0x05, 0xc3, 0x05, 0xc5,
    0x05, 0xc7, 0x05, 0xc9, 0x05, 0xcb, 0x05, 0xcd, 0x05, 0xcf, 0x05, 0xd1, 0x05, 0xd3, 0x05, 0xd5,
    0x05, 0xd7, 0x05, 0xd9, 0x05, 0xdb, 0x05, 0xdd, 0x05, 0xdf, 0x05, 0xe1, 0x05, 0xe3, 0x05, 0xe5,
    0x05, 0xe7, 0x05, 0xe9, 0x05, 0xeb, 0x05, 0xed, 0x05, 0xef, 0x05, 0xf1, 0x05, 0xf3, 0x05, 0xf5,
    0x05, 0xf7, 0x05, 0xf9, 0x05, 0xfb, 0x05, 0xfd, 0x06, 0x00, 0x06, 0x02, 0x06, 0x04, 0x06, 0x06,
    0x06, 0x08, 0x06, 0x0a, 0x06, 0x0c, 0x06, 0x0e, 0x06, 0x10, 0x06, 0x12, 0x06, 0x14, 0x06, 0x16,
    0x06, 0x18, 0x06, 0x1a, 0x06, 0x1c, 0x06, 0x1e, 0x06, 0x20, 0x06, 0x22, 0x06, 0x24, 0x06, 0x26,
    0x06, 0x28, 0x06, 0x2a, 0x06, 0x2c, 0x06, 0x2e, 0x06, 0x30, 0x06, 0x32, 0x06, 0x34, 0x06, 0x36,
    0x06, 0x38, 0x06, 0x3a, 0x06, 0x3c, 0x06, 0x3e, 0x06, 0x40, 0x06, 0x42, 0x06, 0x44, 0x06, 0x46,
    0x06, 0x48, 0x06, 0x4a, 0x06, 0x4c, 0x06, 0x4e, 0x06, 0x50, 0x06, 0x52, 0x06, 0x54, 0x06, 0x56,
    0x06, 0x58, 0x06, 0x5a, 0x06, 0x5c, 0x06, 0x5e, 0x06, 0x60, 0x06, 0x62, 0x06, 0x64, 0x06, 0x66,
    0x06, 0x68, 0x06, 0x6a, 0x06, 0x6c, 0x06, 0x6e, 0x06, 0x70, 0x06, 0x72, 0x06, 0x74, 0x06, 0x76,
    0x06, 0x78,
};

static size_t current_random_index = 0;
static void rand_poll_(uint8_t *buf, uint8_t length) {
  while (length--) {
    if (++current_random_index >= sizeof(test_6_pattern)) {
      current_random_index = 0;
    }
    buf[length] = test_6_pattern[current_random_index];
  }
}

int ble_transport_to_ll_iso_impl(struct os_mbuf *om) {
  /* From my understanding, os_mbuf works similarly to an arena allocator. I iterate over all the
   * nodes and copy them to a continuous chunk of memory */
  size_t len = 1;
  for (struct os_mbuf *m = om; m; m = SLIST_NEXT(m, om_next)) {
    len += m->om_len;
  }

  uint8_t *buf = malloc(len);  // TODO: Don't use malloc

  buf[0] = HCI_H4_ISO;

  size_t i = 1;
  for (struct os_mbuf *m = om; m; m = SLIST_NEXT(m, om_next)) {
    memcpy(&buf[i], m->om_data, m->om_len);
    i += m->om_len;
  }

  int32_t err = sdc_hci_iso_data_put(buf);

  free(buf);

  return err;
}

static void fault_handler_(const char *file, const uint32_t line) {
  return;  // TODO: Log error
}

static void sdc_callback_(void) {
  uint8_t buf[HCI_MSG_BUFFER_MAX_SIZE];
  uint8_t msg_type;

  (void)sdc_hci_get(buf, &msg_type);

  switch (msg_type) {
    case SDC_HCI_MSG_TYPE_EVT:
      ble_transport_to_hs_evt((void *)buf);
      return;
    case SDC_HCI_MSG_TYPE_DATA:
      ble_transport_to_hs_acl((void *)buf);
      return;
    default:
      assert(0);
      break;
  }

  return;
}

void ble_transport_ll_init(void) {
  int32_t err;

  err = mpsl_init(NULL, 25, fault_handler_);
  if (err < 0) {
    return;
  }

  err = sdc_init(fault_handler_);
  if (err < 0) {
    return;
  }

  err = sdc_rand_source_register(&(sdc_rand_source_t){.rand_poll = rand_poll_});
  if (err < 0) {
    return;
  }

  err = sdc_support_adv();
  if (err < 0) {
    return;
  }

  err = sdc_support_peripheral();
  if (err < 0) {
    return;
  }

  const sdc_cfg_t cfg = {.central_count = {.count = 1}};
  err = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG, SDC_CFG_TYPE_CENTRAL_COUNT, &cfg);
  if (err < 0) {
    return;
  }

  uint8_t *mem = malloc(err);

  err = sdc_enable(sdc_callback_, mem);
  if (err < 0) {
    return;
  }

  // free(mem);

  return;
}

int ble_transport_to_ll_acl_impl(struct os_mbuf *om) {
  size_t len = 1;
  for (struct os_mbuf *m = om; m; m = SLIST_NEXT(m, om_next)) {
    len += m->om_len;
  }

  uint8_t *buf = malloc(len);

  buf[0] = HCI_H4_ACL;

  size_t i = 1;
  for (struct os_mbuf *m = om; m; m = SLIST_NEXT(m, om_next)) {
    memcpy(&buf[i], m->om_data, m->om_len);
    i += m->om_len;
  }

  int32_t err = sdc_hci_data_put(buf);

  free(buf);

  return err;
}

int ble_transport_to_ll_cmd_impl(void *buf) {
  struct ble_hci_cmd *cmd = (struct ble_hci_cmd *)buf;

  uint16_t opcode = le16toh(cmd->opcode);
  uint8_t ogf = BLE_HCI_OGF(opcode);
  uint8_t ocf = BLE_HCI_OCF(opcode);

  void *data = (void *)(cmd->data);

  uint8_t err = NRF_EOPNOTSUPP;

  struct ble_hci_ev *hci_ev = (struct ble_hci_ev *)cmd;
  hci_ev->opcode = BLE_HCI_EVCODE_COMMAND_COMPLETE;
  hci_ev->length = sizeof(struct ble_hci_ev_command_complete);
  void *rspbuf = hci_ev->data + sizeof(struct ble_hci_ev_command_complete);

  switch (ogf) {
    case 0x01:
      switch (ocf) {
        case BLE_HCI_OCF_DISCONNECT_CMD:
          err = sdc_hci_cmd_lc_disconnect(data);
          break;

        case BLE_HCI_OCF_RD_REM_VER_INFO:
          err = sdc_hci_cmd_lc_read_remote_version_information(data);
          break;

        default:

          break;
      }
      break;

    case 0x02:
      /*There doesn't seem to be anything for this ogf*/
      break;

    case 0x03:
      switch (ocf) {
        case BLE_HCI_OCF_CB_SET_EVENT_MASK:
          err = sdc_hci_cmd_le_set_event_mask(data);
          break;

        case BLE_HCI_OCF_CB_RESET:
          err = sdc_hci_cmd_cb_reset();
          break;

        case BLE_HCI_OCF_CB_READ_TX_PWR:
          err = sdc_hci_cmd_cb_read_transmit_power_level(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_cb_read_transmit_power_level_return_t);
          break;

        case BLE_HCI_OCF_CB_SET_CTLR_TO_HOST_FC:
          err = sdc_hci_cmd_cb_set_controller_to_host_flow_control(data);
          break;

        case BLE_HCI_OCF_CB_HOST_BUF_SIZE:
          err = sdc_hci_cmd_cb_host_buffer_size(data);
          break;

        case BLE_HCI_OCF_CB_HOST_NUM_COMP_PKTS:
          err = sdc_hci_cmd_cb_host_number_of_completed_packets(data);
          break;

        case BLE_HCI_OCF_CB_SET_EVENT_MASK2:
          err = sdc_hci_cmd_cb_set_event_mask_page_2(data);
          break;

        case BLE_HCI_OCF_CB_RD_AUTH_PYLD_TMO:
          err = sdc_hci_cmd_cb_read_authenticated_payload_timeout(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_cb_read_authenticated_payload_timeout_return_t);
          break;

        case BLE_HCI_OCF_CB_WR_AUTH_PYLD_TMO:
          err = sdc_hci_cmd_cb_write_authenticated_payload_timeout(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_cb_write_authenticated_payload_timeout_return_t);
          break;

        default:

          break;
      }
      break;

    case 0x04:
      switch (ocf) {
        case BLE_HCI_OCF_IP_RD_LOCAL_VER:
          err = sdc_hci_cmd_ip_read_local_version_information(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_ip_read_local_version_information_return_t);
          break;

        case BLE_HCI_OCF_IP_RD_LOC_SUPP_CMD:
          err = sdc_hci_cmd_ip_read_local_supported_commands(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_ip_read_local_supported_commands_return_t);
          break;

        case BLE_HCI_OCF_IP_RD_LOC_SUPP_FEAT:
          err = sdc_hci_cmd_ip_read_local_supported_features(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_ip_read_local_supported_features_return_t);
          break;

        case BLE_HCI_OCF_IP_RD_BUF_SIZE:

          break;

        case BLE_HCI_OCF_IP_RD_BD_ADDR:
          err = sdc_hci_cmd_ip_read_bd_addr(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_ip_read_bd_addr_return_t);
          break;

        default:

          break;
      }
      break;

    case 0x05:
      switch (ocf) {
        case BLE_HCI_OCF_RD_RSSI:
          err = sdc_hci_cmd_sp_read_rssi(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_sp_read_rssi_return_t);
          break;

        default:

          break;
      }
      break;

    case 0x06:
      /*There doesn't seem to be anything for this ogf*/
      break;

    case 0x08:
      switch (ocf) {
        case BLE_HCI_OCF_LE_SET_EVENT_MASK:
          err = sdc_hci_cmd_le_set_event_mask(data);
          break;

        case BLE_HCI_OCF_LE_RD_BUF_SIZE:
          err = sdc_hci_cmd_le_read_buffer_size(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_buffer_size_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_BUF_SIZE_V2:
          err = sdc_hci_cmd_le_read_buffer_size_v2(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_buffer_size_v2_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_LOC_SUPP_FEAT:
          err = sdc_hci_cmd_le_read_local_supported_features(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_local_supported_features_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_RAND_ADDR:
          err = sdc_hci_cmd_le_set_random_address(data);
          break;

        case BLE_HCI_OCF_LE_SET_ADV_PARAMS:
          err = sdc_hci_cmd_le_set_adv_params(data);
          break;

        case BLE_HCI_OCF_LE_RD_ADV_CHAN_TXPWR:
          err = sdc_hci_cmd_le_read_adv_physical_channel_tx_power(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_adv_physical_channel_tx_power_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_ADV_DATA:
          err = sdc_hci_cmd_le_set_adv_data(data);
          break;

        case BLE_HCI_OCF_LE_SET_SCAN_RSP_DATA:
          err = sdc_hci_cmd_le_set_scan_response_data(data);
          break;

        case BLE_HCI_OCF_LE_SET_ADV_ENABLE:
          err = sdc_hci_cmd_le_set_adv_enable(data);
          break;

        case BLE_HCI_OCF_LE_SET_SCAN_PARAMS:
          err = sdc_hci_cmd_le_set_scan_params(data);
          break;

        case BLE_HCI_OCF_LE_SET_SCAN_ENABLE:
          err = sdc_hci_cmd_le_set_scan_enable(data);
          break;

        case BLE_HCI_OCF_LE_CREATE_CONN:
          err = sdc_hci_cmd_le_create_conn(data);
          break;

        case BLE_HCI_OCF_LE_CREATE_CONN_CANCEL:
          err = sdc_hci_cmd_le_create_conn_cancel();
          break;

        case BLE_HCI_OCF_LE_RD_WHITE_LIST_SIZE:
          err = sdc_hci_cmd_le_read_filter_accept_list_size(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_filter_accept_list_size_return_t);
          break;

        case BLE_HCI_OCF_LE_CLEAR_WHITE_LIST:
          err = sdc_hci_cmd_le_clear_filter_accept_list();
          break;

        case BLE_HCI_OCF_LE_ADD_WHITE_LIST:
          err = sdc_hci_cmd_le_add_device_to_filter_accept_list(data);
          break;

        case BLE_HCI_OCF_LE_RMV_WHITE_LIST:
          err = sdc_hci_cmd_le_remove_device_from_filter_accept_list(data);
          break;

        case BLE_HCI_OCF_LE_CONN_UPDATE:
          err = sdc_hci_cmd_le_conn_update(data);
          break;

        case BLE_HCI_OCF_LE_SET_HOST_CHAN_CLASS:
          err = sdc_hci_cmd_le_set_host_channel_classification(data);
          break;

        case BLE_HCI_OCF_LE_RD_CHAN_MAP:
          err = sdc_hci_cmd_le_read_channel_map(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_channel_map_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_REM_FEAT:
          err = sdc_hci_cmd_le_read_remote_features(data);
          break;

        case BLE_HCI_OCF_LE_ENCRYPT:
          err = sdc_hci_cmd_le_encrypt(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_encrypt_return_t);
          break;

        case BLE_HCI_OCF_LE_RAND:
          err = sdc_hci_cmd_le_rand(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_rand_return_t);
          break;

        case BLE_HCI_OCF_LE_START_ENCRYPT:
          err = sdc_hci_cmd_le_enable_encryption(data);
          break;

        case BLE_HCI_OCF_LE_LT_KEY_REQ_REPLY:
          err = sdc_hci_cmd_le_long_term_key_request_reply(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_long_term_key_request_reply_return_t);
          break;

        case BLE_HCI_OCF_LE_LT_KEY_REQ_NEG_REPLY:
          err = sdc_hci_cmd_le_long_term_key_request_negative_reply(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_long_term_key_request_negative_reply_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_SUPP_STATES:
          err = sdc_hci_cmd_le_read_supported_states(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_supported_states_return_t);
          break;

        case BLE_HCI_OCF_LE_RX_TEST:

          break;

        case BLE_HCI_OCF_LE_TX_TEST:

          break;

        case BLE_HCI_OCF_LE_TEST_END:
          break;

        case BLE_HCI_OCF_LE_REM_CONN_PARAM_RR:

          break;

        case BLE_HCI_OCF_LE_REM_CONN_PARAM_NRR:

          break;

        case BLE_HCI_OCF_LE_SET_DATA_LEN:
          err = sdc_hci_cmd_le_set_data_length(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_set_data_length_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_SUGG_DEF_DATA_LEN:
          err = sdc_hci_cmd_le_read_suggested_default_data_length(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_suggested_default_data_length_return_t);
          break;

        case BLE_HCI_OCF_LE_WR_SUGG_DEF_DATA_LEN:
          err = sdc_hci_cmd_le_write_suggested_default_data_length(data);
          break;

        case BLE_HCI_OCF_LE_RD_P256_PUBKEY:

          break;

        case BLE_HCI_OCF_LE_GEN_DHKEY:

          break;

        case BLE_HCI_OCF_LE_ADD_RESOLV_LIST:
          err = sdc_hci_cmd_le_add_device_to_resolving_list(data);
          break;

        case BLE_HCI_OCF_LE_RMV_RESOLV_LIST:
          err = sdc_hci_cmd_le_remove_device_from_resolving_list(data);
          break;

        case BLE_HCI_OCF_LE_CLR_RESOLV_LIST:
          err = sdc_hci_cmd_le_clear_resolving_list();
          break;

        case BLE_HCI_OCF_LE_RD_RESOLV_LIST_SIZE:
          err = sdc_hci_cmd_le_read_resolving_list_size(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_resolving_list_size_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_PEER_RESOLV_ADDR:

          break;

        case BLE_HCI_OCF_LE_RD_LOCAL_RESOLV_ADDR:

          break;

        case BLE_HCI_OCF_LE_SET_ADDR_RES_EN:
          err = sdc_hci_cmd_le_set_address_resolution_enable(data);
          break;

        case BLE_HCI_OCF_LE_SET_RPA_TMO:
          err = sdc_hci_cmd_le_set_resolvable_private_address_timeout(data);
          break;

        case BLE_HCI_OCF_LE_RD_MAX_DATA_LEN:
          err = sdc_hci_cmd_le_read_max_data_length(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_max_data_length_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_PHY:
          err = sdc_hci_cmd_le_read_phy(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_phy_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_DEFAULT_PHY:
          err = sdc_hci_cmd_le_set_default_phy(data);
          break;

        case BLE_HCI_OCF_LE_SET_PHY:
          err = sdc_hci_cmd_le_set_phy(data);
          break;

        case BLE_HCI_OCF_LE_RX_TEST_V2:

          break;

        case BLE_HCI_OCF_LE_TX_TEST_V2:

          break;

        case BLE_HCI_OCF_LE_SET_ADV_SET_RND_ADDR:
          err = sdc_hci_cmd_le_set_adv_set_random_address(data);
          break;

        case BLE_HCI_OCF_LE_SET_EXT_ADV_PARAM:
          err = sdc_hci_cmd_le_set_ext_adv_params(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_set_ext_adv_params_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_EXT_ADV_DATA:
          err = sdc_hci_cmd_le_set_ext_adv_data(data);
          break;

        case BLE_HCI_OCF_LE_SET_EXT_SCAN_RSP_DATA:
          err = sdc_hci_cmd_le_set_ext_scan_response_data(data);
          break;

        case BLE_HCI_OCF_LE_SET_EXT_ADV_ENABLE:
          err = sdc_hci_cmd_le_set_ext_adv_enable(data);
          break;

        case BLE_HCI_OCF_LE_RD_MAX_ADV_DATA_LEN:
          err = sdc_hci_cmd_le_read_max_adv_data_length(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_max_adv_data_length_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_NUM_OF_ADV_SETS:
          err = sdc_hci_cmd_le_read_number_of_supported_adv_sets(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_number_of_supported_adv_sets_return_t);
          break;

        case BLE_HCI_OCF_LE_REMOVE_ADV_SET:
          err = sdc_hci_cmd_le_remove_adv_set(data);
          break;

        case BLE_HCI_OCF_LE_CLEAR_ADV_SETS:
          err = sdc_hci_cmd_le_clear_adv_sets();
          break;

        case BLE_HCI_OCF_LE_SET_PERIODIC_ADV_PARAMS:
          err = sdc_hci_cmd_le_set_periodic_adv_params(data);
          break;

        case BLE_HCI_OCF_LE_SET_PERIODIC_ADV_DATA:
          err = sdc_hci_cmd_le_set_periodic_adv_data(data);
          break;

        case BLE_HCI_OCF_LE_SET_PERIODIC_ADV_ENABLE:
          err = sdc_hci_cmd_le_set_periodic_adv_enable(data);
          break;

        case BLE_HCI_OCF_LE_SET_EXT_SCAN_PARAM:
          err = sdc_hci_cmd_le_set_ext_scan_params(data);
          break;

        case BLE_HCI_OCF_LE_SET_EXT_SCAN_ENABLE:
          err = sdc_hci_cmd_le_set_ext_scan_enable(data);
          break;

        case BLE_HCI_OCF_LE_EXT_CREATE_CONN:
          err = sdc_hci_cmd_le_ext_create_conn(data);
          break;

        case BLE_HCI_OCF_LE_PERIODIC_ADV_CREATE_SYNC:
          err = sdc_hci_cmd_le_periodic_adv_create_sync(data);
          break;

        case BLE_HCI_OCF_LE_PERIODIC_ADV_CREATE_SYNC_CANCEL:
          err = sdc_hci_cmd_le_periodic_adv_create_sync_cancel();
          break;

        case BLE_HCI_OCF_LE_PERIODIC_ADV_TERM_SYNC:
          err = sdc_hci_cmd_le_periodic_adv_terminate_sync(data);
          break;

        case BLE_HCI_OCF_LE_ADD_DEV_TO_PERIODIC_ADV_LIST:
          err = sdc_hci_cmd_le_add_device_to_periodic_adv_list(data);
          break;

        case BLE_HCI_OCF_LE_REM_DEV_FROM_PERIODIC_ADV_LIST:
          err = sdc_hci_cmd_le_remove_device_from_periodic_adv_list(data);
          break;

        case BLE_HCI_OCF_LE_CLEAR_PERIODIC_ADV_LIST:
          err = sdc_hci_cmd_le_clear_periodic_adv_list();
          break;

        case BLE_HCI_OCF_LE_RD_PERIODIC_ADV_LIST_SIZE:
          err = sdc_hci_cmd_le_read_periodic_adv_list_size(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_periodic_adv_list_size_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_TRANSMIT_POWER:
          err = sdc_hci_cmd_le_read_transmit_power(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_transmit_power_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_RF_PATH_COMPENSATION:
          err = sdc_hci_cmd_le_read_rf_path_compensation(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_rf_path_compensation_return_t);
          break;

        case BLE_HCI_OCF_LE_WR_RF_PATH_COMPENSATION:
          err = sdc_hci_cmd_le_write_rf_path_compensation(data);
          break;

        case BLE_HCI_OCF_LE_SET_PRIVACY_MODE:
          err = sdc_hci_cmd_le_set_privacy_mode(data);
          break;

        case BLE_HCI_OCF_LE_RX_TEST_V3:

          break;

        case BLE_HCI_OCF_LE_TX_TEST_V3:

          break;

        case BLE_HCI_OCF_LE_SET_CONNLESS_CTE_TX_PARAMS:
          err = sdc_hci_cmd_le_set_connless_cte_transmit_params(data);
          break;

        case BLE_HCI_OCF_LE_SET_CONNLESS_CTE_TX_ENABLE:
          err = sdc_hci_cmd_le_set_connless_cte_transmit_enable(data);
          break;

        case BLE_HCI_OCF_LE_SET_CONNLESS_IQ_SAMPLING_ENABLE:

          break;

        case BLE_HCI_OCF_LE_SET_CONN_CTE_RX_PARAMS:

          break;

        case BLE_HCI_OCF_LE_SET_CONN_CTE_TX_PARAMS:
          err = sdc_hci_cmd_le_set_conn_cte_transmit_params(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_set_conn_cte_transmit_params_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_CONN_CTE_REQ_ENABLE:

          break;

        case BLE_HCI_OCF_LE_SET_CONN_CTE_RESP_ENABLE:
          err = sdc_hci_cmd_le_conn_cte_response_enable(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_conn_cte_response_enable_return_t);
          break;

        case BLE_HCI_OCF_LE_RD_ANTENNA_INFO:
          err = sdc_hci_cmd_le_read_antenna_information(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_antenna_information_return_t);
          break;

        case BLE_HCI_OCF_LE_PERIODIC_ADV_RECEIVE_ENABLE:
          err = sdc_hci_cmd_le_set_periodic_adv_receive_enable(data);
          break;

        case BLE_HCI_OCF_LE_PERIODIC_ADV_SYNC_TRANSFER:
          err = sdc_hci_cmd_le_periodic_adv_sync_transfer(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_periodic_adv_sync_transfer_return_t);
          break;

        case BLE_HCI_OCF_LE_PERIODIC_ADV_SET_INFO_TRANSFER:
          err = sdc_hci_cmd_le_periodic_adv_set_info_transfer(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_periodic_adv_set_info_transfer_return_t);
          break;

        case BLE_HCI_OCF_LE_PERIODIC_ADV_SYNC_TRANSFER_PARAMS:
          err = sdc_hci_cmd_le_set_periodic_adv_sync_transfer_params(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_set_periodic_adv_sync_transfer_params_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_DEFAULT_SYNC_TRANSFER_PARAMS:
          err = sdc_hci_cmd_le_set_default_periodic_adv_sync_transfer_params(data);
          break;

        case BLE_HCI_OCF_LE_GENERATE_DHKEY_V2:

          break;

        case BLE_HCI_OCF_LE_MODIFY_SCA:

          break;

        case BLE_HCI_OCF_LE_READ_ISO_TX_SYNC:
          err = sdc_hci_cmd_le_read_iso_tx_sync(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_iso_tx_sync_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_CIG_PARAMS:
          err = sdc_hci_cmd_le_set_cig_params(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_set_cig_params_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_CIG_PARAMS_TEST:
          err = sdc_hci_cmd_le_set_cig_params_test(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_set_cig_params_test_return_t);
          break;

        case BLE_HCI_OCF_LE_CREATE_CIS:
          err = sdc_hci_cmd_le_create_cis(data);
          break;

        case BLE_HCI_OCF_LE_REMOVE_CIG:
          err = sdc_hci_cmd_le_remove_cig(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_remove_cig_return_t);
          break;

        case BLE_HCI_OCF_LE_ACCEPT_CIS_REQ:
          err = sdc_hci_cmd_le_accept_cis_request(data);
          break;

        case BLE_HCI_OCF_LE_REJECT_CIS_REQ:
          err = sdc_hci_cmd_le_reject_cis_request(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_reject_cis_request_return_t);
          break;

        case BLE_HCI_OCF_LE_CREATE_BIG:
          err = sdc_hci_cmd_le_create_big(data);
          break;

        case BLE_HCI_OCF_LE_CREATE_BIG_TEST:
          err = sdc_hci_cmd_le_create_big_test(data);
          break;

        case BLE_HCI_OCF_LE_TERMINATE_BIG:
          err = sdc_hci_cmd_le_terminate_big(data);
          break;

        case BLE_HCI_OCF_LE_BIG_CREATE_SYNC:
          err = sdc_hci_cmd_le_big_create_sync(data);
          break;

        case BLE_HCI_OCF_LE_BIG_TERMINATE_SYNC:
          err = sdc_hci_cmd_le_big_terminate_sync(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_big_terminate_sync_return_t);
          break;

        case BLE_HCI_OCF_LE_REQ_PEER_SCA:
          err = sdc_hci_cmd_le_request_peer_sca(data);
          break;

        case BLE_HCI_OCF_LE_SETUP_ISO_DATA_PATH:
          err = sdc_hci_cmd_le_setup_iso_data_path(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_setup_iso_data_path_return_t);
          break;

        case BLE_HCI_OCF_LE_REMOVE_ISO_DATA_PATH:
          err = sdc_hci_cmd_le_remove_iso_data_path(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_remove_iso_data_path_return_t);
          break;

        case BLE_HCI_OCF_LE_ISO_TRANSMIT_TEST:
          err = sdc_hci_cmd_le_iso_transmit_test(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_iso_transmit_test_return_t);
          break;

        case BLE_HCI_OCF_LE_ISO_RECEIVE_TEST:
          err = sdc_hci_cmd_le_iso_receive_test(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_iso_receive_test_return_t);
          break;

        case BLE_HCI_OCF_LE_ISO_READ_TEST_COUNTERS:
          err = sdc_hci_cmd_le_iso_read_test_counters(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_iso_read_test_counters_return_t);
          break;

        case BLE_HCI_OCF_LE_ISO_TEST_END:
          err = sdc_hci_cmd_le_iso_test_end(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_iso_test_end_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_HOST_FEATURE:
          err = sdc_hci_cmd_le_set_host_feature(data);
          break;

        case BLE_HCI_OCF_LE_READ_ISO_LINK_QUALITY:
          err = sdc_hci_cmd_le_read_iso_link_quality(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_read_iso_link_quality_return_t);
          break;

        case BLE_HCI_OCF_LE_ENH_READ_TRANSMIT_POWER_LEVEL:
          err = sdc_hci_cmd_le_enhanced_read_transmit_power_level(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_enhanced_read_transmit_power_level_return_t);
          break;

        case BLE_HCI_OCF_LE_READ_REMOTE_TRANSMIT_POWER_LEVEL:
          err = sdc_hci_cmd_le_read_remote_transmit_power_level(data);
          break;

        case BLE_HCI_OCF_LE_SET_PATH_LOSS_REPORT_PARAM:
          err = sdc_hci_cmd_le_set_path_loss_reporting_params(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_set_path_loss_reporting_params_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_PATH_LOSS_REPORT_ENABLE:
          err = sdc_hci_cmd_le_set_path_loss_reporting_enable(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_set_path_loss_reporting_enable_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_TRANS_PWR_REPORT_ENABLE:
          err = sdc_hci_cmd_le_set_transmit_power_reporting_enable(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_set_transmit_power_reporting_enable_return_t);
          break;

        case BLE_HCI_OCF_LE_SET_DEFAULT_SUBRATE:
          err = sdc_hci_cmd_le_set_default_subrate(data);
          break;

        case BLE_HCI_OCF_LE_SUBRATE_REQ:
          err = sdc_hci_cmd_le_subrate_request(data);
          break;

        case BLE_HCI_OCF_LE_SET_EXT_ADV_PARAM_V2:
          err = sdc_hci_cmd_le_set_ext_adv_params_v2(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_set_ext_adv_params_v2_return_t);
          break;

        case BLE_HCI_OCF_LE_CS_RD_LOC_SUPP_CAP:
          err = sdc_hci_cmd_le_cs_read_local_supported_capabilities(rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_cs_read_local_supported_capabilities_return_t);
          break;

        case BLE_HCI_OCF_LE_CS_RD_REM_SUPP_CAP:
          err = sdc_hci_cmd_le_cs_read_remote_supported_capabilities(data);
          break;

        case BLE_HCI_OCF_LE_CS_WR_CACHED_REM_SUPP_CAP:
          err = sdc_hci_cmd_le_cs_write_cached_remote_supported_capabilities(data, rspbuf);
          hci_ev->length +=
              sizeof(sdc_hci_cmd_le_cs_write_cached_remote_supported_capabilities_return_t);
          break;

        case BLE_HCI_OCF_LE_CS_SEC_ENABLE:
          err = sdc_hci_cmd_le_cs_security_enable(data);
          break;

        case BLE_HCI_OCF_LE_CS_SET_DEF_SETTINGS:
          err = sdc_hci_cmd_le_cs_set_default_settings(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_cs_set_default_settings_return_t);
          break;

        case BLE_HCI_OCF_LE_CS_RD_REM_FAE:
          err = sdc_hci_cmd_le_cs_read_remote_fae_table(data);
          break;

        case BLE_HCI_OCF_LE_CS_WR_CACHED_REM_FAE:
          err = sdc_hci_cmd_le_cs_write_cached_remote_fae_table(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_cs_write_cached_remote_fae_table_return_t);
          break;

        case BLE_HCI_OCF_LE_CS_CREATE_CONFIG:
          err = sdc_hci_cmd_le_cs_create_config(data);
          break;

        case BLE_HCI_OCF_LE_CS_REMOVE_CONFIG:
          err = sdc_hci_cmd_le_cs_remove_config(data);
          break;

        case BLE_HCI_OCF_LE_CS_SET_CHAN_CLASS:
          err = sdc_hci_cmd_le_cs_set_channel_classification(data);
          break;

        case BLE_HCI_OCF_LE_CS_SET_PROC_PARAMS:
          err = sdc_hci_cmd_le_cs_set_procedure_params(data, rspbuf);
          hci_ev->length += sizeof(sdc_hci_cmd_le_cs_set_procedure_params_return_t);
          break;

        case BLE_HCI_OCF_LE_CS_PROC_ENABLE:
          err = sdc_hci_cmd_le_cs_procedure_enable(data);
          break;

        case BLE_HCI_OCF_LE_CS_TEST:
          err = sdc_hci_cmd_le_cs_test(data);
          break;

        case BLE_HCI_OCF_LE_CS_TEST_END:
          err = sdc_hci_cmd_le_cs_test_end();
          break;

        default:

          break;
      }
      break;

    case 0x3F:

      switch (ocf) {
        case BLE_HCI_OCF_VS_RD_STATIC_ADDR:

          break;

        case BLE_HCI_OCF_VS_SET_TX_PWR:

          break;

        case BLE_HCI_OCF_VS_CSS_CONFIGURE:

          break;

        case BLE_HCI_OCF_VS_CSS_ENABLE:

          break;

        case BLE_HCI_OCF_VS_CSS_SET_NEXT_SLOT:

          break;

        case BLE_HCI_OCF_VS_CSS_SET_CONN_SLOT:

          break;

        case BLE_HCI_OCF_VS_CSS_READ_CONN_SLOT:

          break;

        case BLE_HCI_OCF_VS_SET_DATA_LEN:

          break;

        case BLE_HCI_OCF_VS_SET_ANTENNA:

          break;

        case BLE_HCI_OCF_VS_SET_LOCAL_IRK:

          break;

        case BLE_HCI_OCF_VS_SET_SCAN_CFG:

          break;

        default:

          break;
      }

      break;

    default:

      break;
  }

  struct ble_hci_ev_command_complete *cmd_complete = (void *)hci_ev->data;
  cmd_complete->status = 0;
  cmd_complete->num_packets = 1;
  cmd_complete->opcode = htole16(opcode);

  ble_transport_to_hs_evt(hci_ev);

  // ble_transport_free(buf);
  return -((int)(err));
}
