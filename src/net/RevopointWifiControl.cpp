#include "RevopointWifiControl.h"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>
#include <QUrl>

namespace revoscan::net {

RevopointWifiControl::RevopointWifiControl(QObject* parent)
    : QObject(parent), nam_(new QNetworkAccessManager(this)) {}

RevopointWifiControl::~RevopointWifiControl() = default;

void RevopointWifiControl::setHost(const QString& host, quint16 port) {
    host_.host = host.toStdString();
    host_.port = port;
}

QString RevopointWifiControl::host() const {
    return QString::fromStdString(host_.host);
}

void RevopointWifiControl::send(const std::string& query, const QString& tag) {
    const QString url = QString::fromStdString(zxCmdUrl(host_, query));
    QNetworkRequest request{QUrl(url)};
    request.setAttribute(QNetworkRequest::RedirectPolicyAttribute,
                         QNetworkRequest::NoLessSafeRedirectPolicy);
    QNetworkReply* reply = nam_->get(request);

    // Per-request timeout: abort the reply, which triggers finished() with an
    // OperationCanceledError we translate into a failure.
    auto* timer = new QTimer(reply);
    timer->setSingleShot(true);
    connect(timer, &QTimer::timeout, reply, [reply] {
        if (reply->isRunning()) {
            reply->abort();
        }
    });
    timer->start(timeoutMs_);

    connect(reply, &QNetworkReply::finished, this, [this, reply, tag] {
        reply->deleteLater();
        if (reply->error() != QNetworkReply::NoError) {
            const QString err = reply->errorString();
            emit commandResult(tag, false, err);
            if (tag == QStringLiteral("get_sn")) {
                emit probed(false, err);
            }
            return;
        }

        const QByteArray body = reply->readAll();
        const std::string bodyStr = body.toStdString();
        emit commandResult(tag, true, QString::fromUtf8(body));

        if (tag == QStringLiteral("get_sn")) {
            const auto serial = parseSerialResponse(bodyStr);
            emit probed(true,
                        serial ? QString::fromStdString(*serial)
                               : QStringLiteral("(reachable, empty serial)"));
        } else if (tag == QStringLiteral("get_exposure")) {
            if (const auto value = parseIntResponse(bodyStr)) {
                emit exposureRead(static_cast<int>(*value));
            }
        }
    });
}

void RevopointWifiControl::probe() {
    send(zxGetSerial(), QStringLiteral("get_sn"));
}

void RevopointWifiControl::getExposure() {
    send(zxGetExposure(), QStringLiteral("get_exposure"));
}

void RevopointWifiControl::setExposure(int microseconds) {
    send(zxSetExposure(microseconds), QStringLiteral("set_exposure"));
}

void RevopointWifiControl::getGain() {
    send(zxGetGain(), QStringLiteral("get_gain"));
}

void RevopointWifiControl::setRedLightIndicator(bool on) {
    send(zxSetRedLightIndicator(on), QStringLiteral("set_RedLightIndicator"));
}

void RevopointWifiControl::sendRaw(const QString& query, const QString& tag) {
    send(query.toStdString(), tag);
}

} // namespace revoscan::net
