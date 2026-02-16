(ns app.components.app
  (:require [uix.core :refer [defui $]]
            [app.components.sidebar :as sidebar]
            [app.components.map :as map-view]))

(defui app []
  ($ :div {:class "app-root"}
    ($ sidebar/sidebar)
    ($ :div {:class "map-container"}
      ($ map-view/route-map))))
