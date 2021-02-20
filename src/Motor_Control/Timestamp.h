
// Create a time for Time to be used on Time related operations
typedef uint32_t Timestamp;

/// Allow using human readable units

inline static constexpr const Timestamp operator"" _seconds(unsigned long long const x) { return x * 1000; }
inline static constexpr const Timestamp operator"" _seconds(long double const x) { return x * 1000; }
inline static constexpr const Timestamp operator"" _minutes(unsigned long long const x) { return x * 60_seconds; }
inline static constexpr const Timestamp operator"" _minutes(long double const x) { return x * 60_seconds; }
inline static constexpr const Timestamp operator"" _hours(unsigned long long const x) { return x * 60_minutes; }
inline static constexpr const Timestamp operator"" _hours(long double const x) { return x * 60_minutes; }