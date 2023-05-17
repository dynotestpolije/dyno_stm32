BLK=`tput setaf 0`
RED=`tput setaf 9`
GRN=`tput setaf 2`
YLW=`tput setaf 3`
BLU=`tput setaf 4`
CYN=`tput setaf 6`
WHT=`tput setaf 7`
BLD=`tput bold`
RST=`tput sgr0`
perror() { ARG="$1"; shift; ARGS="$@"; printf "${RED}${BLD}INFO:     ${RST}${RED}${ARG}${RST} ${ARGS}" >&2 ; }

