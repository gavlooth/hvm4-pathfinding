(ns app.api
  (:require [app.state :as state]))

(def BASE "http://localhost:8080")

(defn post! [path body]
  (-> (js/fetch (str BASE path)
        #js {:method "POST"
             :headers #js {"Content-Type" "application/json"}
             :body (js/JSON.stringify (clj->js body))})
      (.then #(.json %))
      (.then #(js->clj % :keywordize-keys true))))

(defn get! [path]
  (-> (js/fetch (str BASE path))
      (.then #(.json %))
      (.then #(js->clj % :keywordize-keys true))))

(defn make-weather-grid
  "Generate a synthetic weather grid for given hour offset.
   Wave height and wind speed scale with hour."
  [hour]
  (let [rows 5 cols 5
        n (* rows cols)
        base-wave (+ 0.5 (* hour 0.05))
        base-wind (+ 3.0 (* hour 0.3))]
    {:sw [37.0 23.0]
     :ne [39.0 27.0]
     :rows rows
     :cols cols
     :wind_speed (vec (repeat n base-wind))
     :wind_dir   (vec (repeat n 180.0))
     :wave_height (vec (repeat n base-wave))
     :wave_dir    (vec (repeat n 180.0))
     :current_speed (vec (repeat n 0.2))
     :current_dir   (vec (repeat n 90.0))
     :timestamp (double hour)}))

(defn push-weather! [hour]
  (post! "/api/weather" (make-weather-grid hour)))

(defn route! []
  (reset! state/loading true)
  (-> (post! "/api/route" {:algorithm @state/algorithm})
      (.then (fn [data]
               (reset! state/current-route data)
               (reset! state/loading false)))
      (.catch (fn [e]
                (js/console.error "Route error:" e)
                (reset! state/loading false)))))

(defn setup-and-route!
  "Full pipeline: init -> voyage -> weather -> route"
  []
  (reset! state/loading true)
  (reset! state/current-route nil)
  (reset! state/selected-waypoint nil)
  (let [{:keys [ship-speed max-wave]} @state/ship-config
        {:keys [origin destination]} @state/voyage
        hour @state/weather-hour]
    (-> (post! "/api/init"
          {:ship_speed ship-speed
           :max_wave max-wave
           :land {:sw [37.0 23.0] :ne [39.0 27.0]
                  :rows 5 :cols 5 :grid []}
           :prm {:n_samples 200 :k_neighbors 8 :seed 42}})
        (.then (fn [data]
                 (swap! state/app-state assoc
                   :initialized true
                   :node-count (:node_count data))
                 (post! "/api/voyage" {:origin origin :destination destination})))
        (.then (fn [_] (push-weather! hour)))
        (.then (fn [_] (route!)))
        (.catch (fn [e]
                  (js/console.error "Setup error:" e)
                  (reset! state/loading false))))))

(defn reroute-weather!
  "Push new weather and re-route (called when weather slider changes)"
  [hour]
  (when (:initialized @state/app-state)
    (-> (post! "/api/weather/clear" {})
        (.then (fn [_] (push-weather! hour)))
        (.then (fn [_] (route!)))
        (.catch #(js/console.error "Reroute error:" %)))))
