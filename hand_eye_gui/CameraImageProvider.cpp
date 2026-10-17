#include "CameraImageProvider.hpp"

CameraImageProvider::CameraImageProvider()
    : QQuickImageProvider(QQuickImageProvider::Image)
{
    setPlaceholder(640, 480);
}

QImage CameraImageProvider::requestImage(const QString& /*id*/,
                                          QSize* size,
                                          const QSize& /*requestedSize*/)
{
    QMutexLocker lock(&m_mutex);
    if (size)
        *size = m_image.size();
    return m_image.copy();  // 返回副本，避免 QML 端与写入端竞争
}

void CameraImageProvider::setImage(const QImage& img)
{
    QMutexLocker lock(&m_mutex);
    m_image = img.copy();
}

void CameraImageProvider::setPlaceholder(int width, int height)
{
    QImage placeholder(width, height, QImage::Format_RGB888);
    placeholder.fill(QColor(0x0d, 0x11, 0x17));  // 深色背景
    m_image = placeholder;
}
