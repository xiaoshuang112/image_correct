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
    // QImage 隐式共享持有当前帧；setImage 替换缓冲区不影响已交给 QML 的帧。
    return m_image;
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
