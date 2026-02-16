(ns app.components.waypoint
  (:require [uix.core :refer [defui $]]
            [app.state :as state]))

(defui waypoint-panel []
  (let [wp (state/use-atom state/selected-waypoint)]
    (when wp
      ($ :div {:class "waypoint-panel"}
        ($ :button {:class "close-btn"
                    :on-click #(reset! state/selected-waypoint nil)}
          "\u00D7")
        ($ :h3 (str "Waypoint #" (:node_id wp)))
        ($ :div {:class "info-row"}
          ($ :span {:class "label"} "Position")
          ($ :span {:class "value"}
            (str (.toFixed (:lat wp) 4) ", " (.toFixed (:lon wp) 4))))
        ($ :div {:class "info-row"}
          ($ :span {:class "label"} "ETA")
          ($ :span {:class "value"} (str (.toFixed (:eta_hours wp) 2) " hours")))
        ($ :div {:class "info-row"}
          ($ :span {:class "label"} "Speed")
          ($ :span {:class "value"} (str (.toFixed (:speed_knots wp) 1) " kt")))
        ($ :div {:class "info-row"}
          ($ :span {:class "label"} "Wave Height")
          ($ :span {:class "value"} (str (.toFixed (:wave_height wp) 1) " m")))
        ($ :div {:class "info-row"}
          ($ :span {:class "label"} "Wind Speed")
          ($ :span {:class "value"} (str (.toFixed (:wind_speed wp) 1) " kt")))
        ($ :div {:class "info-row"}
          ($ :span {:class "label"} "Heading")
          ($ :span {:class "value"} (str (.toFixed (:heading_deg wp) 0) "\u00B0")))))))
