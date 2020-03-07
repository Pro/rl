//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <QHostAddress>
#include <QStatusBar>
#include <QTextStream>
#include <rl/math/Rotation.h>
#include <rl/sg/Body.h>
#include <rl/sg/Shape.h>

#include "ConfigurationModel.h"
#include "ConnectedModel.h"
#include "MainWindow.h"
#include "Socket.h"

Socket::Socket(QObject* parent) :
	QTcpSocket(parent)
{
	QObject::connect(this, SIGNAL(disconnected()), this, SLOT(deleteLater()));
	QObject::connect(this, SIGNAL(readyRead()), this, SLOT(readClient()));
}

Socket::~Socket()
{
}

void
Socket::readClient()
{
	MainWindow::instance()->statusBar()->showMessage("Received data from " + this->peerAddress().toString() + ":" + QString::number(this->peerPort()), 1000);
	
	QTextStream textStream(this);
	
	for (QString line = textStream.readLine(); QString() != line; line = textStream.readLine())
	{
		QStringList list = line.split(" ");
		
		if (list.size() < 1)
		{
			continue;
		}
		
		std::size_t cmd = list[0].toUInt();
		
		switch (cmd)
		{
		case 0:
			{
				if (9 != list.size())
				{
					continue;
				}
				
				std::size_t i = list[1].toUInt();
				std::size_t j = list[2].toUInt();
				rl::math::Real x = list[3].toDouble();
				rl::math::Real y = list[4].toDouble();
				rl::math::Real z = list[5].toDouble();
				rl::math::Real a = list[6].toDouble();
				rl::math::Real b = list[7].toDouble();
				rl::math::Real c = list[8].toDouble();
				
				rl::math::Transform t;
				t = rl::math::AngleAxis(c, rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(b, rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(a, rl::math::Vector3::UnitX());
				t.translation().x() = x;
				t.translation().y() = y;
				t.translation().z() = z;
				
				if (MainWindow::instance()->scene->getNumModels() > i)
				{
					if (MainWindow::instance()->scene->getModel(i)->getNumBodies() > j)
					{
						MainWindow::instance()->scene->getModel(i)->getBody(j)->setFrame(t);
					}
				}
			}
			break;
		case 1:
			{
				if (10 != list.size())
				{
					continue;
				}
				
				std::size_t i = list[1].toUInt();
				std::size_t j = list[2].toUInt();
				std::size_t k = list[3].toUInt();
				rl::math::Real x = list[4].toDouble();
				rl::math::Real y = list[5].toDouble();
				rl::math::Real z = list[6].toDouble();
				rl::math::Real a = list[7].toDouble();
				rl::math::Real b = list[8].toDouble();
				rl::math::Real c = list[9].toDouble();
				
				rl::math::Transform t;
				t = rl::math::AngleAxis(c, rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(b, rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(a, rl::math::Vector3::UnitX());
				t.translation().x() = x;
				t.translation().y() = y;
				t.translation().z() = z;
				
				if (MainWindow::instance()->scene->getNumModels() > i)
				{
					if (MainWindow::instance()->scene->getModel(i)->getNumBodies() > j)
					{
						if (MainWindow::instance()->scene->getModel(i)->getBody(j)->getNumShapes() > k)
						{
							MainWindow::instance()->scene->getModel(i)->getBody(j)->getShape(k)->setTransform(t);
						}
					}
				}
			}
			break;
		case 2:
			{
				if (list.size() < 2)
				{
					continue;
				}
				
				std::size_t i = list[1].toUInt();
				
				if (2 + MainWindow::instance()->kinematicModels[i]->getDofPosition() != list.size())
				{
					continue;
				}
				
				if (i < MainWindow::instance()->kinematicModels.size())
				{
					rl::math::Vector q(MainWindow::instance()->kinematicModels[i]->getDofPosition());
					q.setZero();
					
					for (std::ptrdiff_t j = 0; j < q.size(); ++j)
					{
						q(j) = list[2 + j].toDouble();
					}
					
					MainWindow::instance()->configurationModels[i]->setData(q);
				}

                MainWindow::instance()->connectedModel->updateConnections();
			}
			break;
		case 6:
			{
				textStream << cmd;
				
				if (list.size() < 2)
				{
					textStream << endl;
					continue;
				}
				
				std::size_t i = list[1].toUInt();
				textStream << " " << i;
				
				if (i < MainWindow::instance()->kinematicModels.size())
				{
					rl::math::Vector q = MainWindow::instance()->kinematicModels[i]->getPosition();
					
					for (std::size_t i = 0; i < q.size(); ++i)
					{
						textStream << " " << q(i);
					}
				}
				
				textStream << endl;
			}
			break;
        case 20:
            {
                // connect endeffector with body
                // Message Format:
                // 20 kinIdx effIdx modIdx bodIdx x y z a b c uuid
                // kinIdx = index of the kinematic model
                // effIdx = index of the endeffector
                // modIdx = index of the model to be connected
                // bodIdx = index of the body inside the model
                // x,y,z,a,b,c = optional offset between endeffector and body. If not given, current offset is used
                // uuid = unique identifier which can be used to disconnect the body

                if (12 != list.size() && 6 != list.size()) {
                    continue;
                }

                const std::size_t kinIdx = list[1].toUInt();
                const std::size_t effIdx = list[2].toUInt();
                const std::size_t modIdx = list[3].toUInt();
                const std::size_t bodIdx = list[4].toUInt();

                if (kinIdx >= MainWindow::instance()->kinematicModels.size() ||
                    effIdx >= MainWindow::instance()->kinematicModels[kinIdx]->getOperationalDof() ||
                    modIdx >= MainWindow::instance()->scene->getNumModels() ||
                    bodIdx >= MainWindow::instance()->scene->getModel(modIdx)->getNumBodies()) {
                    continue;
                }

                rl::math::Transform offset;
                std::string uuid;
                if (list.size() == 12) {
                    const rl::math::Real x = list[5].toDouble();
                    const rl::math::Real y = list[6].toDouble();
                    const rl::math::Real z = list[7].toDouble();
                    const rl::math::Real a = list[8].toDouble();
                    const rl::math::Real b = list[9].toDouble();
                    const rl::math::Real c = list[10].toDouble();
                    uuid = list[11].toStdString();

                    offset = rl::math::AngleAxis(c, rl::math::Vector3::UnitZ()) *
                             rl::math::AngleAxis(b, rl::math::Vector3::UnitY()) *
                             rl::math::AngleAxis(a, rl::math::Vector3::UnitX());
                    offset.translation().x() = x;
                    offset.translation().y() = y;
                    offset.translation().z() = z;
                } else {
                    uuid = list[5].toStdString();

                    rl::math::Transform effPos = MainWindow::instance()->kinematicModels[kinIdx]->getOperationalPosition(effIdx);
                    rl::math::Transform modelPos;
                    MainWindow::instance()->scene->getModel(modIdx)->getBody(bodIdx)->getFrame(modelPos);

                    offset = effPos.inverse() * modelPos;
                }

                MainWindow::instance()->connectedModel->addConnection(kinIdx, effIdx, modIdx, bodIdx, offset, uuid);
            }
            break;
        case 21:
            {
                if (2 != list.size()) {
                    continue;
                }
                const std::string uuid = list[1].toStdString();
                MainWindow::instance()->connectedModel->removeConnection(uuid);
                break;
            }
		default:
			break;
		}
	}
}
