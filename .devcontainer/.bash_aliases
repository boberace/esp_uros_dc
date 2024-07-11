export LC_ALL=C
. /opt/esp/idf/export.sh

alias sba="source ~/.bash_aliases"

# show git git branch in prompt
parse_git_branch() {
     git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1)/'
}
export PS1="\u@\h \[\e[32m\]\w \[\e[91m\]\$(parse_git_branch)\[\e[00m\]$ "

# run inside micro_ros_espidf_component folder
function urosesp(){
	docker run -it --rm --user espidf \
	--volume="/etc/timezone:/etc/timezone:ro" \
	--volume  $(pwd):/micro_ros_espidf_component \
	--volume  /dev:/dev \
	--privileged \
	--workdir /micro_ros_espidf_component \
	--group-add dialout \
	microros/esp-idf-microros:latest \
	/bin/bash
}