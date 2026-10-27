#pragma once

#include <QString>

namespace collection {

// Per-image JSON has only x/y/z/rx/ry/rz (mm, axis-angle degrees).
// A null/non-finite pose writes six nulls. The first valid pose in a daily
// directory also creates T_base_flange.json (translation in metres).
// Returns warnings; a record failure never removes or changes the image.
QString saveFlangePoseRecords(const QString& imagePath, const double* pose);

// Standalone TCP/flange JSON uses the same six fields (mm, SDK orientation in degrees).
// It does not create or update the daily flange matrix.
QString saveStandalonePose(const QString& directory, const QString& base, const double* pose, QString& savedPath);

} // namespace collection
