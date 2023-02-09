/*
 * Copyright (C) 2023 Rhys Mainwaring
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>
#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <gz/msgs/double.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif
#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "test_config.hh"
#include "../../../../test/helpers/EnvTestFixture.hh"

#include "../../GuiRunner.hh"

#include "ForceDisplay.hh"

int g_argc = 1;
char* g_argv[] =
{
  reinterpret_cast<char*>(const_cast<char*>("dummy")),
};

using namespace gz;

/// \brief Tests for the joint position controller GUI plugin
class ForceDisplay : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(ForceDisplay, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(Load))
{
  // Create app
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);
  app->AddPluginPath(std::string(PROJECT_BINARY_PATH) + "/lib");

  // Create GUI runner to handle sim::gui plugins
  auto runner = new sim::GuiRunner("test");
  runner->setParent(gui::App());

  // Add plugin
  const char *pluginStr =
    "<plugin filename=\"ForceDisplay\">"
      "<gz-gui>"
        "<title>ForceDisplay!</title>"
      "</gz-gui>"
      "<maximum_points>123</maximum_points>"
      "<update_rate>5</update_rate>"
    "</plugin>";

  tinyxml2::XMLDocument pluginDoc;
  EXPECT_EQ(tinyxml2::XML_SUCCESS, pluginDoc.Parse(pluginStr));
  EXPECT_TRUE(app->LoadPlugin("ForceDisplay",
      pluginDoc.FirstChildElement("plugin")));

  // Get main window
  auto win = app->findChild<gui::MainWindow *>();
  ASSERT_NE(nullptr, win);

  // Get plugin
  auto plugins = win->findChildren<sim::gui::ForceDisplay *>();
  ASSERT_EQ(plugins.size(), 1);

  auto plugin = plugins[0];
  EXPECT_EQ("ForceDisplay!", plugin->Title());
  // EXPECT_EQ(sim::kNullEntity, plugin->TargetEntity());
  // EXPECT_EQ(QString("banana"), plugin->TargetName())
  //     << plugin->TargetName().toStdString();
  EXPECT_EQ(5, plugin->UpdateRate());
  // EXPECT_TRUE(plugin->Locked());

  // Cleanup
  plugins.clear();
}
