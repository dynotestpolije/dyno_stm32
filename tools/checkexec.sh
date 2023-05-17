#!/usr/bin/env bash
source $(dirname $0)/incl.sh

EXEC_NAME="$1";
_ARGS="${BLU}${EXEC_NAME}${RST}"
_COLS=100
STATUS_CHECK=$(command -v $EXEC_NAME >/dev/null 2>&1 && printf "${GRN}✅ Installed${RST}" || printf "❌ ${RED}Not Installed${RST}");
printf "${BLD}[CHECK EXECUTABLE] => ${_ARGS} %*s\n" $((_COLS - ${#_ARGS})) "${STATUS_CHECK}"; 

