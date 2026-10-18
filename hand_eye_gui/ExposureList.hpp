#pragma once

#include <QRegularExpression>
#include <QString>
#include <set>
#include <utility>
#include <vector>

namespace collection {

enum class ExposureListError { None, InvalidValue, InvalidCount };

// Keep input order; leave out unchanged on failure. Check every value before
// the count so mixed value/count errors retain their original priority.
inline ExposureListError parseExposureList(const QString& text, std::vector<int>& out) {
    std::vector<int> values;
    std::set<int> distinct;
    const auto tokens = text.split(QRegularExpression("[,，;\\s]+"), Qt::SkipEmptyParts);
    for (const auto& token : tokens) {
        bool ok = false;
        const int value = token.toInt(&ok);
        if (!ok || value < 1 || value > 1000000 || !distinct.insert(value).second)
            return ExposureListError::InvalidValue;
        values.push_back(value);
    }
    if (values.size() < 2 || values.size() > 5)
        return ExposureListError::InvalidCount;
    out = std::move(values);
    return ExposureListError::None;
}

} // namespace collection
