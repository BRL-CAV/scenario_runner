//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#pragma once

#include "log.h"

#define MAX_LOG_MESSAGE_LENGTH 120

#define LOG_MESSAGE_TAG "LOG "

struct LogMessage
{
    enum LogLevel log_level;
    enum LogSource log_source;
    char log_text[MAX_LOG_MESSAGE_LENGTH];
};
