//
// Created by jemin on 11/4/19.
// MIT License
//
// Copyright (c) 2019-2019 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef RAISIMOGRE_A1_IMGUI_RENDER_CALLBACK_HPP
#define RAISIMOGRE_A1_IMGUI_RENDER_CALLBACK_HPP

#include "raisim/imgui_plot.h"
#include "font.hpp"

namespace raisim {
namespace robot_gui {

static std::vector<std::function<void()>> callbackList;

void init(const std::vector<std::function<void()>>& callback_list) {
  callbackList = callback_list;
}

void robotImguiRenderCallBack() {

  ImGui::SetNextWindowPos({0, 0});

  ImGui::Begin("imgui_panel", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

  auto vis = raisim::OgreVis::get();
  auto world = vis->getWorld();
  unsigned long mask = 0;

  // if (ImGui::CollapsingHeader("Video recording")) {
  //   if(vis->isRecording()) {
  //     ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.8f, 0.2f, 1.f));
  //     ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.9f, 0.3f, 1.f));
  //     ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.5f, 0.9f, 0.5f, 1.f));

  //     if(ImGui::Button("Stop Recording ")) {
  //       RSINFO("Stop recording")
  //       raisim::OgreVis::get()->stopRecordingVideoAndSave();
  //     }

  //     ImGui::PopStyleColor(3);
  //   } else {
  //     ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.2f, 1.f));
  //     ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.3f, 0.3f, 1.f));
  //     ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.9f, 0.5f, 0.5f, 1.f));

  //     if(ImGui::Button("Record ")){
  //       RSINFO("Start recording")
  //       raisim::OgreVis::get()->startRecordingVideo(raisim::OgreVis::get()->getResourceDir() + "/test.mp4");
  //     }

  //     ImGui::PopStyleColor(3);
  //   }
  // }



  if (ImGui::CollapsingHeader("Visualization")) {
    ImGui::PushFont(fontBig);
    ImGui::PopFont();

    ImGui::PushFont(fontMid);
    ImGui::Checkbox("Bodies", &raisim::gui::showBodies);
    ImGui::Checkbox("Collision Bodies", &raisim::gui::showCollision);
    ImGui::Checkbox("Contact Points", &raisim::gui::showContacts);
    ImGui::Checkbox("Contact Forces", &raisim::gui::showForces);

    ImGui::PopFont();
  }

    if(raisim::gui::showBodies) mask |= raisim::OgreVis::RAISIM_OBJECT_GROUP;
    if(raisim::gui::showCollision) mask |= raisim::OgreVis::RAISIM_COLLISION_BODY_GROUP;
    if(raisim::gui::showContacts) mask |= raisim::OgreVis::RAISIM_CONTACT_POINT_GROUP;
    if(raisim::gui::showForces) mask |= raisim::OgreVis::RAISIM_CONTACT_FORCE_GROUP;

  // if (ImGui::CollapsingHeader("Simulation")) {
  //   ImGui::PushFont(fontMid);
  //   ImGui::Text("Sim time: %8.3f, Time step: %8.3f", world->getWorldTime(), world->getTimeStep());
  //   static int takeNSteps = 1;
  //   ImGui::Checkbox("Manual stepping", &raisim::gui::manualStepping);
  //   if(raisim::gui::manualStepping) {
  //     std::string tempString = "Remaining Steps: " + std::to_string(vis->getTakeNSteps());
  //     ImGui::Text("%s", tempString.c_str());
  //     ImGui::Text("Take "); ImGui::SameLine(); ImGui::InputInt("", &takeNSteps); ImGui::SameLine(); ImGui::Text(" steps"); ImGui::SameLine();
  //     if(ImGui::Button("Run"))
  //       vis->getTakeNSteps() += takeNSteps;
  //   } else {
  //     if(ImGui::Button("Set to real time"))
  //       vis->getRealTimeFactorReference() = 1.f;
  //     ImGui::SameLine();
  //     ImGui::SliderFloat("", &vis->getRealTimeFactorReference(), 1e-3, 1000, "Real time factor %5.4f", 10);
  //   }
  //   ImGui::PopFont();
  // }

  vis->setVisibilityMask(mask);


  
  for(auto call : callbackList) {
    call();
    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 10.0f));
  }

  ImGui::End();
}


void imguiSetupCallback() {

#define HIL(v)   ImVec4(0.002f, 0.075f, 0.0356f, v)
#define MEDL(v)  ImVec4(0.555f, 0.598f, 0.501f, v)
#define LOWL(v)  ImVec4(0.232f, 0.201f, 0.21f, v)
  // backgrounds (@todo: complete with BG_MEDL, BG_LOWL)
#define BG(v)   ImVec4(0.200f, 0.220f, 0.270f, v)
  // text
#define TEXT(v) ImVec4(0.860f, 0.930f, 0.890f, v)

  auto &style = ImGui::GetStyle();
  style.Alpha = 0.8;
  style.Colors[ImGuiCol_Text]                  = TEXT(0.78f);
  style.Colors[ImGuiCol_TextDisabled]          = TEXT(0.28f);
  style.Colors[ImGuiCol_WindowBg]              = ImVec4(0.13f, 0.14f, 0.17f, 0.7f);
  style.Colors[ImGuiCol_ChildWindowBg]         = BG( 0.38f);
  style.Colors[ImGuiCol_PopupBg]               = BG( 0.3f);
  style.Colors[ImGuiCol_Border]                = ImVec4(0.31f, 0.31f, 1.00f, 0.00f);
  style.Colors[ImGuiCol_BorderShadow]          = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  style.Colors[ImGuiCol_FrameBg]               = BG( 1.00f);
  style.Colors[ImGuiCol_FrameBgHovered]        = MEDL( 0.78f);
  style.Colors[ImGuiCol_FrameBgActive]         = MEDL( 1.00f);
  style.Colors[ImGuiCol_TitleBg]               = LOWL( 1.00f);
  style.Colors[ImGuiCol_TitleBgActive]         = HIL( 1.00f);
  style.Colors[ImGuiCol_TitleBgCollapsed]      = BG( 0.75f);
  style.Colors[ImGuiCol_MenuBarBg]             = BG( 0.47f);
  style.Colors[ImGuiCol_ScrollbarBg]           = BG( 1.00f);
  style.Colors[ImGuiCol_ScrollbarGrab]         = ImVec4(0.09f, 0.15f, 0.16f, 1.00f);
  style.Colors[ImGuiCol_ScrollbarGrabHovered]  = MEDL( 0.78f);
  style.Colors[ImGuiCol_ScrollbarGrabActive]   = MEDL( 1.00f);
  style.Colors[ImGuiCol_CheckMark]             = ImVec4(0.71f, 0.22f, 0.27f, 1.00f);
  style.Colors[ImGuiCol_SliderGrab]            = ImVec4(0.47f, 0.77f, 0.83f, 0.14f);
  style.Colors[ImGuiCol_SliderGrabActive]      = ImVec4(0.71f, 0.22f, 0.27f, 1.00f);
  style.Colors[ImGuiCol_Button]                = ImVec4(0.47f, 0.77f, 0.83f, 0.14f);
  style.Colors[ImGuiCol_ButtonHovered]         = MEDL( 0.86f);
  style.Colors[ImGuiCol_ButtonActive]          = MEDL( 1.00f);
  style.Colors[ImGuiCol_Header]                = MEDL( 0.76f);
  style.Colors[ImGuiCol_HeaderHovered]         = MEDL( 0.86f);
  style.Colors[ImGuiCol_HeaderActive]          = HIL( 1.00f);
  style.Colors[ImGuiCol_Column]                = ImVec4(0.14f, 0.16f, 0.19f, 1.00f);
  style.Colors[ImGuiCol_ColumnHovered]         = MEDL( 0.78f);
  style.Colors[ImGuiCol_ColumnActive]          = MEDL( 1.00f);
  style.Colors[ImGuiCol_ResizeGrip]            = ImVec4(0.47f, 0.77f, 0.83f, 0.04f);
  style.Colors[ImGuiCol_ResizeGripHovered]     = MEDL( 0.78f);
  style.Colors[ImGuiCol_ResizeGripActive]      = MEDL( 1.00f);
  style.Colors[ImGuiCol_PlotLines]             = TEXT(0.63f);
  style.Colors[ImGuiCol_PlotLinesHovered]      = MEDL( 1.00f);
  style.Colors[ImGuiCol_PlotHistogram]         = TEXT(0.63f);
  style.Colors[ImGuiCol_PlotHistogramHovered]  = MEDL( 1.00f);
  style.Colors[ImGuiCol_TextSelectedBg]        = MEDL( 0.43f);
  // [...]
  style.Colors[ImGuiCol_ModalWindowDarkening]  = BG( 0.53f);

  style.WindowPadding            = ImVec2(6, 4);
  style.WindowRounding           = 0.0f;
  style.FramePadding             = ImVec2(5, 2);
  style.FrameRounding            = 3.0f;
  style.ItemSpacing              = ImVec2(7, 1);
  style.ItemInnerSpacing         = ImVec2(1, 1);
  style.TouchExtraPadding        = ImVec2(0, 0);
  style.IndentSpacing            = 6.0f;
  style.ScrollbarSize            = 12.0f;
  style.ScrollbarRounding        = 16.0f;
  style.GrabMinSize              = 20.0f;
  style.GrabRounding             = 2.0f;

  style.WindowTitleAlign.x = 0.50f;

  style.Colors[ImGuiCol_Border] = ImVec4(0.839f, 0.879f, 0.855f, 0.162f);
  style.FrameBorderSize = 0.0f;
  style.WindowBorderSize = 1.0f;

  ImGuiIO &io = ImGui::GetIO();
  fontBig = io.Fonts->AddFontFromFileTTF((raisim::OgreVis::get()->getResourceDir() + "/font/DroidSans.ttf").c_str(), 25.0f);
  fontMid = io.Fonts->AddFontFromFileTTF((raisim::OgreVis::get()->getResourceDir() + "/font/DroidSans.ttf").c_str(), 22.0f);
  fontSmall = io.Fonts->AddFontFromFileTTF((raisim::OgreVis::get()->getResourceDir() + "/font/DroidSans.ttf").c_str(), 16.0f);
}

}
}


#endif //RAISIMOGRE_ANYMAL_IMGUI_RENDER_CALLBACK_HPP
