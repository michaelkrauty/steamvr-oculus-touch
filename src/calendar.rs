// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Calendar dates of times, in UTC, for logs and archives.

use std::time::{SystemTime, UNIX_EPOCH};

/// A time in UTC: year, month (1-12), day (1-31), hour, minute, second and millisecond.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Utc {
    pub year: i64,
    pub month: u32,
    pub day: u32,
    pub hour: u32,
    pub minute: u32,
    pub second: u32,
    pub millisecond: u32,
}

impl Utc {
    /// `at` in UTC (a time before 1970 counts as 1970).
    pub fn of(at: SystemTime) -> Utc {
        let since = at.duration_since(UNIX_EPOCH).unwrap_or_default();
        let (days, second) = ((since.as_secs() / 86_400) as i64, since.as_secs() % 86_400);
        // the civil date of a day count (Howard Hinnant's days_from_civil, inverted)
        let z = days + 719_468;
        let era = z.div_euclid(146_097);
        let day_of_era = z.rem_euclid(146_097);
        let year_of_era =
            (day_of_era - day_of_era / 1_460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
        let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
        let shifted_month = (5 * day_of_year + 2) / 153;
        let day = day_of_year - (153 * shifted_month + 2) / 5 + 1;
        let month = if shifted_month < 10 {
            shifted_month + 3
        } else {
            shifted_month - 9
        };
        Utc {
            year: year_of_era + era * 400 + i64::from(month <= 2),
            month: month as u32,
            day: day as u32,
            hour: (second / 3600) as u32,
            minute: (second / 60 % 60) as u32,
            second: (second % 60) as u32,
            millisecond: since.subsec_millis(),
        }
    }

    /// 2026-10-14T17:05:03.123Z
    pub fn iso(&self) -> String {
        format!(
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}.{:03}Z",
            self.year, self.month, self.day, self.hour, self.minute, self.second, self.millisecond
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn times_are_utc_to_the_millisecond() {
        let at = |secs: u64, ms: u64| UNIX_EPOCH + Duration::from_millis(secs * 1000 + ms);
        assert_eq!(Utc::of(at(0, 0)).iso(), "1970-01-01T00:00:00.000Z");
        assert_eq!(
            Utc::of(at(951_782_400, 5)).iso(),
            "2000-02-29T00:00:00.005Z"
        );
        assert_eq!(
            Utc::of(at(1_791_997_503, 123)).iso(),
            "2026-10-14T17:05:03.123Z"
        );
        assert_eq!(
            Utc::of(at(4_102_444_799, 999)).iso(),
            "2099-12-31T23:59:59.999Z"
        );
    }
}
