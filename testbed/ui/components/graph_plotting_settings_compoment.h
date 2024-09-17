//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef GRAPH_PLOTTING_SETTINGS_COMPOMENT_H
#define GRAPH_PLOTTING_SETTINGS_COMPOMENT_H
#include "imgui.h"
#include "plot/plot.h"
#include "ui/ui_component.h"

class GraphPlottingSettingsComponent final : public UIComponent {
private:
  bool plot_graphs;
  bool plot_graph1;
  bool plot_graph2;
  bool plot_graph3;
  bool plot_graph4;
  bool plot_graph5;

public:
  GraphPlottingSettingsComponent()
      : plot_graphs(true), plot_graph1(false), plot_graph2(false),
        plot_graph3(false), plot_graph4(false), plot_graph5(false) {}

  // Render the component
  void render() override {
    ImGui::SeparatorText("Graph Plotting Settings");

    // Checkbox to enable or disable all graphs
    ImGui::Checkbox("Plot Graphs", &plot_graphs);

    if (plot_graphs) {
      ImGui::Checkbox("Plot Targets Found", &testbed::plot_targets_found);
      ImGui::Checkbox("Plot Drone Distances", &testbed::plot_drone_distances);
      ImGui::Checkbox("Plot Drone Heatmap", &testbed::plot_drone_heatmap);
      ImGui::Checkbox("Plot Drone Trace", &testbed::plot_drone_trace);
      ImGui::Checkbox("Plot Drone Speed", &testbed::plot_drone_speed);
    } else {
      plot_graph1 = false;
      plot_graph2 = false;
      plot_graph3 = false;
      plot_graph4 = false;
      plot_graph5 = false;
    }
  }
};

#endif //GRAPH_PLOTTING_SETTINGS_COMPOMENT_H
