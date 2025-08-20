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

# Functions to trunc numbers when logging
function get_exponent_sciform(number::Real)
    # Get the exponent of a number in scientific notation
    return number == 0.0 ? 0 : Int(floor(log10(abs(number))))
end
function round_from_tol(n::Real, tol::Real, rshift::Int)
    # Round a number n to the same number of digits as the tolerance (+ shift on the right)
    return round(n, digits = (-1) * get_exponent_sciform(tol) + rshift)
end

# Functions to round / reformat timing records
function tidy_timing(time::Real)
    return round(time, digits = 3)
end