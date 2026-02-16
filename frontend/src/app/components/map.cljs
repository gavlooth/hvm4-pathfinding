(ns app.components.map
  (:require [uix.core :as uix :refer [defui $]]
            [app.state :as state]
            ["react-leaflet" :refer [MapContainer TileLayer Polyline CircleMarker Popup useMap]]))

(defn interpolate-position [waypoints progress]
  (let [n (count waypoints)]
    (when (and (pos? n) (some? progress))
      (let [idx-f (* progress (dec n))
            idx (int idx-f)
            idx (min idx (- n 2))
            t (- idx-f idx)
            wp1 (nth waypoints idx)
            wp2 (nth waypoints (inc idx))]
        #js [(+ (* (- 1 t) (:lat wp1)) (* t (:lat wp2)))
             (+ (* (- 1 t) (:lon wp1)) (* t (:lon wp2)))]))))

(defui fit-bounds []
  (let [route (state/use-atom state/current-route)
        map-ref (useMap)]
    (uix/use-effect
      (fn []
        (when-let [wps (:waypoints route)]
          (when (seq wps)
            (let [lats (map :lat wps)
                  lons (map :lon wps)]
              (.fitBounds map-ref
                (clj->js [[(apply min lats) (apply min lons)]
                          [(apply max lats) (apply max lons)]])
                #js {:padding #js [50 50]})))))
      [route])
    nil))

(defui route-map []
  (let [route (state/use-atom state/current-route)
        progress (state/use-atom state/animation-progress)
        wps (:waypoints route)
        positions (when (seq wps)
                    (clj->js (mapv (fn [wp] [(:lat wp) (:lon wp)]) wps)))
        vessel-pos (when (and (seq wps) (some? progress))
                     (interpolate-position wps progress))]
    ($ MapContainer {:center #js [38.0 25.0]
                     :zoom 7
                     :style #js {:height "100%" :width "100%"}}
      ($ TileLayer {:url "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                    :attribution "&copy; OpenStreetMap contributors"})
      ($ fit-bounds)
      (when positions
        ($ Polyline {:positions positions
                     :color "#e94560"
                     :weight 3}))
      (when (seq wps)
        (for [[idx wp] (map-indexed vector wps)]
          ($ CircleMarker
            {:key idx
             :center #js [(:lat wp) (:lon wp)]
             :radius 6
             :pathOptions #js {:fillColor "#4fc3f7"
                               :fillOpacity 0.8
                               :color "#fff"
                               :weight 1}
             :eventHandlers #js {:click (fn [_] (reset! state/selected-waypoint wp))}}
            ($ Popup (str "Node " (:node_id wp))))))
      (when vessel-pos
        ($ CircleMarker
          {:center vessel-pos
           :radius 8
           :pathOptions #js {:fillColor "#e94560"
                             :fillOpacity 1.0
                             :color "#fff"
                             :weight 2}})))))
