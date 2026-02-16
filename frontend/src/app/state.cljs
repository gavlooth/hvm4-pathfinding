(ns app.state
  (:require [uix.core :as uix]))

;; Global atoms
(defonce app-state         (atom {:initialized false :node-count 0}))
(defonce current-route     (atom nil))
(defonce selected-waypoint (atom nil))
(defonce weather-hour      (atom 0))
(defonce ship-config       (atom {:ship-speed 15.0 :max-wave 4.0}))
(defonce algorithm         (atom 2))
(defonce voyage            (atom {:origin [37.5 23.5] :destination [38.5 26.5]}))
(defonce animation-progress (atom nil))
(defonce loading           (atom false))

;; React 18 useSyncExternalStore hook for atoms
(defn use-atom [a]
  (let [subscribe (uix/use-callback
                    (fn [callback]
                      (let [key (gensym "use-atom")]
                        (add-watch a key (fn [_ _ _ _] (callback)))
                        (fn [] (remove-watch a key))))
                    [a])
        snapshot (uix/use-callback #(deref a) [a])]
    (js/React.useSyncExternalStore subscribe snapshot)))
