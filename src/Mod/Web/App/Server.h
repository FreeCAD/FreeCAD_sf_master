/***************************************************************************
 *   Copyright (c) 2014 Werner Mayer <wmayer[at]users.sourceforge.net>     *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <QByteArray>
#include <QObject>
#include <QEvent>
#include <QTcpSocket>
#include <QTcpServer>


namespace Web {

class ServerEvent : public QEvent
{
public:
    ServerEvent(QTcpSocket* socket, const QByteArray&);
    ~ServerEvent();

    QTcpSocket* socket() const;
    const QByteArray& request() const;

private:
    QTcpSocket* sock;
    QByteArray text;
};

/** 
 * The Server class implements a simple TCP server.
 */
class AppServer : public QTcpServer
{
    Q_OBJECT

public:
    AppServer(QObject* parent = 0);

    void incomingConnection(int socket);

protected:
    void customEvent(QEvent* e);

private Q_SLOTS:
    void readClient();
    void discardClient();

private:
};

}

#endif //Web_SERVER_H
