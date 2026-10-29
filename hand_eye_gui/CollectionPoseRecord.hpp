#pragma once

#include <QString>

namespace collection {

// Per-image JSON has only x/y/z/rx/ry/rz (mm, axis-angle degrees).
// A null/non-finite pose writes six nulls. The first valid pose in a daily
// directory also creates T_base_flange.json (translation in metres).
// Returns warnings; a record failure never removes or changes the image.
QString saveFlangePoseRecords(const QString& imagePath, const double* pose);

// Standalone TCP JSON adds tcp_name and the six-value tcp_offset used for this read.
// It does not create or update the daily flange matrix. Units: mm and SDK degrees.
QString saveStandaloneTcpPose(const QString& directory, const QString& base, const double* pose,
                             const QString& tcpName, const double* tcpOffset, QString& savedPath);

} // namespace collection
