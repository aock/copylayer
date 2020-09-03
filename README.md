# copylayer

Costmap2D-Layer. Copy a costmap into another costmaps layer.

```yaml
global_costmap:
  update_frequency: 10
  publish_frequency: 1
  static_map: true
  rolling_window: false
  resolution: 1.0
  plugins:
  - name: static_map
    type: "costmap_2d::StaticLayer"
  - name: inflation
    type: "costmap_2d::InflationLayer"
  - name: copy
    type: "copylayer::CopyLayer"

  copy:
    source: local_costmap
    persistent: false
    enabled: true

  inflation:
    cost_scaling_factor: 4.0
    enabled: true
    inflation_radius: 1.0
```