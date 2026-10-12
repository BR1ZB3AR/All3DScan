#pragma once

#include <QObject>
#include <QString>

#include <cstdint>

#include "net/ZxCmdProtocol.h"

class QNetworkAccessManager;
class QNetworkReply;

namespace revoscan::net {

// Async control channel to a Revopoint scanner in WiFi host mode over the
// embedded zx_cmd.cgi HTTP interface. Streaming frames over WiFi is a separate
// (not yet reverse-engineered) TCP channel; this covers control only.
class RevopointWifiControl : public QObject {
    Q_OBJECT
public:
    explicit RevopointWifiControl(QObject* parent = nullptr);
    ~RevopointWifiControl() override;

    void setHost(const QString& host, quint16 port = 80);
    QString host() const;

    void setTimeoutMs(int ms) { timeoutMs_ = ms; }

    // Reachability probe: fetches the serial number.
    void probe();

    void getExposure();
    void setExposure(int microseconds);
    void getGain();
    void setRedLightIndicator(bool on);

    // Raw escape hatch for experimentation with new query strings.
    void sendRaw(const QString& query, const QString& tag);

signals:
    // ok=true with the serial on success; ok=false with an error string.
    void probed(bool ok, const QString& info);
    // Result of any command: the tag identifies it (e.g. "get_exposure").
    void commandResult(const QString& tag, bool ok, const QString& response);
    void exposureRead(int microseconds);

private:
    void send(const std::string& query, const QString& tag);

    QNetworkAccessManager* nam_ = nullptr;
    ZxHost host_;
    int timeoutMs_ = 4000;
};

} // namespace revoscan::net
