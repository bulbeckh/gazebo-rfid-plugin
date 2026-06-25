#!/bin/bash

gz service --timeout 10000 -s /rfid-scanner/scan_request --reqtype gz.msgs.Empty --reptype gz.custom_msgs.RFIDScanResponse -r ''
