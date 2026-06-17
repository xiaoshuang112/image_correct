#ifndef CAMERA_IMAGE_PROVIDER_HPP
#define CAMERA_IMAGE_PROVIDER_HPP

#include <QQuickImageProvider>
#include <QImage>
#include <QMutex>

/**
 * @brief 将 cv::Mat 帧以 image://camera/current 提供给 QML
 *
 * QML 用法：
 *   Image { source: "image://camera/current"; cache: false }
 */
class CameraImageProvider : public QQuickImageProvider {
public:
    CameraImageProvider();

    QImage requestImage(const QString& id, QSize* size,
                        const QSize& requestedSize) override;

    /// 线程安全地更新当前帧
    void setImage(const QImage& img);

    /// 设置一个占位图像（"等待视频流..."）
    void setPlaceholder(int width, int height);

private:
    QImage  m_image;
    QMutex  m_mutex;
};

#endif // CAMERA_IMAGE_PROVIDER_HPP
