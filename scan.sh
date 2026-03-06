#!/bin/bash

gz service --timeout 10000 -s /scan_request --reqtype gz.msgs.Empty --reptype gz.custom_msgs.RFIDScanResponse -r ''
