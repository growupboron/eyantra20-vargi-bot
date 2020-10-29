#!/usr/bin/env bash


# Exit the script in case of error
set -e

FIREFOX=$(which firefox)

HIVEMQ="http://www.hivemq.com/demos/websocket-client/"
GOOGLE_SHEET="https://docs.google.com/spreadsheets/d/1l8NgRkF_eJNapbHCIWYu06JhW-9NyF-dBeGVYddOjYs/edit?skip_itp2_check=true#gid=0"

firefox -new-window  "$HIVEMQ" &
firefox -new-window "$GOOGLE_SHEET" &






