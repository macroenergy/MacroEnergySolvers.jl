const DATE_FORMAT = "yyyy-mm-dd HH:MM:SS"

### functions for logging
function log_formatter(level::LogLevel, _module, group, id, file, line)
    (color, _, suffix) = Logging.default_metafmt(level, _module, group, id, file, line)
    return color, "$(Dates.format(now(), DATE_FORMAT))", suffix
end

function set_logger(log_level::LogLevel = Logging.Info)
    logger = ConsoleLogger(stdout, log_level, meta_formatter = log_formatter)
    global_logger(logger)
end

set_logger(logger::AbstractLogger) = global_logger(logger)