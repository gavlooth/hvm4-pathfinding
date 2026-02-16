(ns app.core
  (:require [uix.dom :as uix.dom]
            [app.components.app :as app]))

(defonce root (atom nil))

(defn init []
  (when-not @root
    (reset! root (uix.dom/create-root (js/document.getElementById "root"))))
  (.render @root (app/app)))
