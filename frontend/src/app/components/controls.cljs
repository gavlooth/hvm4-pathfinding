(ns app.components.controls
  (:require [uix.core :as uix :refer [defui $]]
            [app.state :as state]
            [app.api :as api]))

(def algorithms
  [{:id 0 :name "APSP"}
   {:id 1 :name "Enumerate"}
   {:id 2 :name "Dijkstra"}
   {:id 3 :name "ALT"}
   {:id 4 :name "CCH"}
   {:id 5 :name "Hub-Label"}])

(defui algorithm-picker []
  (let [alg (state/use-atom state/algorithm)]
    ($ :div {:class "control-group"}
      ($ :label "Algorithm")
      ($ :select {:value alg
                  :on-change #(reset! state/algorithm (js/parseInt (.. % -target -value)))}
        (for [{:keys [id name]} algorithms]
          ($ :option {:key id :value id} name))))))

(defui ship-config-panel []
  (let [{:keys [ship-speed max-wave]} (state/use-atom state/ship-config)]
    ($ :div {:class "control-group"}
      ($ :label "Ship Speed (knots)")
      ($ :span {:class "range-value"} (.toFixed ship-speed 1))
      ($ :input {:type "range" :min 5 :max 25 :step 0.5
                 :value ship-speed
                 :on-change #(swap! state/ship-config assoc :ship-speed
                               (js/parseFloat (.. % -target -value)))})
      ($ :label "Max Wave Height (m)")
      ($ :span {:class "range-value"} (.toFixed max-wave 1))
      ($ :input {:type "range" :min 1 :max 8 :step 0.5
                 :value max-wave
                 :on-change #(swap! state/ship-config assoc :max-wave
                               (js/parseFloat (.. % -target -value)))}))))

(defui voyage-inputs []
  (let [{:keys [origin destination]} (state/use-atom state/voyage)
        update-coord (fn [field idx val]
                       (swap! state/voyage update field assoc idx (js/parseFloat val)))]
    ($ :div {:class "control-group"}
      ($ :label "Origin")
      ($ :div {:class "input-row"}
        ($ :div
          ($ :label "Lat")
          ($ :input {:type "number" :step 0.1 :value (get origin 0)
                     :on-change #(update-coord :origin 0 (.. % -target -value))}))
        ($ :div
          ($ :label "Lon")
          ($ :input {:type "number" :step 0.1 :value (get origin 1)
                     :on-change #(update-coord :origin 1 (.. % -target -value))})))
      ($ :label "Destination")
      ($ :div {:class "input-row"}
        ($ :div
          ($ :label "Lat")
          ($ :input {:type "number" :step 0.1 :value (get destination 0)
                     :on-change #(update-coord :destination 0 (.. % -target -value))}))
        ($ :div
          ($ :label "Lon")
          ($ :input {:type "number" :step 0.1 :value (get destination 1)
                     :on-change #(update-coord :destination 1 (.. % -target -value))}))))))

(defui weather-slider []
  (let [hour (state/use-atom state/weather-hour)]
    ($ :div {:class "control-group"}
      ($ :label "Weather Forecast Hour")
      ($ :span {:class "range-value"} (str "+" hour "h"))
      ($ :input {:type "range" :min 0 :max 48 :step 6
                 :value hour
                 :on-change (fn [e]
                              (let [h (js/parseInt (.. e -target -value))]
                                (reset! state/weather-hour h)
                                (api/reroute-weather! h)))}))))

(defui route-info []
  (let [route (state/use-atom state/current-route)]
    (when route
      ($ :div {:class "route-info"}
        ($ :div {:class "info-row"}
          ($ :span {:class "label"} "Cost")
          ($ :span {:class "value"} (.toFixed (or (:cost_real route) 0) 2)))
        ($ :div {:class "info-row"}
          ($ :span {:class "label"} "Waypoints")
          ($ :span {:class "value"} (str (:path_length route))))
        (when-let [wps (:waypoints route)]
          (when (seq wps)
            (let [last-wp (last wps)]
              ($ :div {:class "info-row"}
                ($ :span {:class "label"} "ETA")
                ($ :span {:class "value"} (str (.toFixed (:eta_hours last-wp) 1) " hours"))))))))))

(defui animate-button []
  (let [progress (state/use-atom state/animation-progress)
        route (state/use-atom state/current-route)
        animating? (some? progress)]
    (when route
      ($ :button
        {:class "animate-btn"
         :on-click (fn []
                     (if animating?
                       (reset! state/animation-progress nil)
                       (let [start (atom nil)
                             duration 10000]
                         (reset! state/animation-progress 0.0)
                         (letfn [(tick [ts]
                                   (when (nil? @start) (reset! start ts))
                                   (let [elapsed (- ts @start)
                                         p (min 1.0 (/ elapsed duration))]
                                     (reset! state/animation-progress p)
                                     (when (< p 1.0)
                                       (js/requestAnimationFrame tick))))]
                           (js/requestAnimationFrame tick)))))}
        (if animating? "Stop Animation" "Animate Vessel")))))

(defui controls []
  (let [loading? (state/use-atom state/loading)]
    ($ :div {:class (if loading? "loading" "")}
      ($ algorithm-picker)
      ($ ship-config-panel)
      ($ voyage-inputs)
      ($ weather-slider)
      ($ :button {:class "route-btn"
                  :disabled loading?
                  :on-click api/setup-and-route!}
        (if loading? "Computing..." "Compute Route"))
      ($ route-info)
      ($ animate-button))))
