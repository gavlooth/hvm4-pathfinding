(ns app.components.sidebar
  (:require [uix.core :refer [defui $]]
            [app.components.controls :as controls]
            [app.components.waypoint :as waypoint]))

(defui sidebar []
  ($ :div {:class "sidebar"}
    ($ :h2 "Maritime Router")
    ($ controls/controls)
    ($ waypoint/waypoint-panel)))
