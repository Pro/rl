//
// Copyright (c) 2020, Stefan Profanter
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

#include "ConnectedModel.h"
#include "MainWindow.h"
#include "rl/sg/Body.h"

ConnectedModel::ConnectedModel() {

}

ConnectedModel::~ConnectedModel() {

}

void ConnectedModel::addConnection(
        size_t kinematicIdx,
        size_t endeffectorIdx,
        size_t modelIdx,
        size_t bodyIdx,
        const rl::math::Transform& offset,
        const std::string& uuid
) {

    if (this->connections.count(uuid) > 0) {
        removeConnection(uuid);
    }

    struct Connection c = {
            .kinematicIdx = kinematicIdx,
            .endeffectorIdx = endeffectorIdx,
            .modelIdx = modelIdx,
            .bodyIdx = bodyIdx,
            .offset = offset
    };

    {
        const std::lock_guard<std::mutex> lock(connectionsMutex);
        this->connections.emplace(uuid, c);
    }
    updateConnections();
}

void ConnectedModel::removeConnection(const std::string& uuid) {
    const std::lock_guard<std::mutex> lock(connectionsMutex);

    if (this->connections.count(uuid) == 0)
        return;
    this->connections.erase(uuid);

}

void ConnectedModel::updateConnections() {
    const std::lock_guard<std::mutex> lock(connectionsMutex);

    for (auto & connection : connections)
    {
        const std::string& uuid = connection.first;
        const struct Connection& c = connection.second;

        if (MainWindow::instance()->kinematicModels.size() <= c.kinematicIdx ||
                MainWindow::instance()->kinematicModels[c.kinematicIdx]->getOperationalDof() <= c.endeffectorIdx)
            continue;

        rl::math::Transform effPos = MainWindow::instance()->kinematicModels[c.kinematicIdx]->getOperationalPosition(c.endeffectorIdx);
        rl::math::Transform t = effPos * c.offset;

        if (MainWindow::instance()->scene->getNumModels() > c.modelIdx)
        {
            if (MainWindow::instance()->scene->getModel(c.modelIdx)->getNumBodies() > c.bodyIdx)
            {
                MainWindow::instance()->scene->getModel(c.modelIdx)->getBody(c.bodyIdx)->setFrame(t);
            }
        }
    }
}
