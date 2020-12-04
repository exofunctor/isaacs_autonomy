# The principal environment variables.
export WORKSPACE=$HOME/ISAACS
# If a valid workspace was given, automatically source it and cd into it.
[[ -e $WORKSPACE ]] && source $WORKSPACE/devel/setup.bash && cd $WORKSPACE
export CONFIG=$HOME/ISAACS-config
[[ -e $CONFIG/fzf ]] && export FZF=$CONFIG/fzf && source $FZF/shell/completion.bash && source $FZF/shell/key-bindings.bash




# Set the text editor to `(n)vim`.
if [[ -e /usr/bin/nvim ]]; then
    export EDITOR="nvim"
elif [[ -e /usr/bin/vim ]]; then
    export EDITOR="vim"
fi




stty -ixon                                 # Prevent ^Q and ^S from freezing the shell.
set -o ignoreeof                           # Prevent ^D from closing the shell.
shopt -s autocd                            # Automatically cd to the entered directory.
echo -en "\e[5 q"                          # Set the cursor to i-beam.



# Quickly navigate to parent directories.
alias ...='cd ../..'
alias ....='cd ../../..'
alias .....='cd ../../../..'
alias ......='cd ../../../../..'
alias .......='cd ../../../../../..'




# PREVIEW FILES
# If `bat` is present, use it as the default pager instead of `less`.
if [[ -e /usr/bin/bat || -e $HOME/.cargo/bin/bat ]]; then
    export PAGER="bat"
    export BAT_THEME="ansi-dark"
fi




# FIND FILES
# Regex warper.
function easy_find
{
    if [[ $2 ]]; then
        find -L $2 -regex ".*$1.*"
    else
        find -L . -regex ".*$1.*"
    fi
}
alias find='easy_find'
# If `fd` is present, use it as the default finder instead of `find`.
if [[ -e /usr/bin/fdfind ]]; then
    # On Debian systems `fd` is named `fdfind` and needs to be aliased.
    export FINDER="fdfind"
    alias fd="fdfind --follow --hidden"
    alias find="fdfind --follow --hidden"

elif [[ -e /usr/bin/fd ]]; then
    export FINDER="fd"
    alias fd="fd --follow --hidden"
    alias find="fd --follow --hidden"

else
    export FINDER="easy_find"
    alias fd='easy_find'
fi




# LIST FILES
# Use `exa` if installed.
if [[ -e /usr/bin/exa || -e $HOME/.cargo/bin/exa ]]; then

    # If the current terminal is urxvt only display text.
    if [[ $TERM == rxvt-unicode-256color ]]; then
        alias l='exa -lFa --time-style=long-iso --color-scale --group-directories-first --git'
        alias ll='exa -lFa --time-style=long-iso --color-scale --group-directories-first --git --tree --level=2'
    # Else, also display icons.
    else
        alias l='exa -lFai --time-style=long-iso --color-scale --icons --group-directories-first --git'
        alias ll='exa -lFai --time-style=long-iso --color-scale --icons --group-directories-first --git --tree --level=2'
    fi

# Else, default to `ls`.
else
    alias l='ls -lFAih --group-directories-first --color=auto'
    alias ll='ls -lFAihR  --group-directories-first --color=auto --dereference-command-line-symlink-to-dir'
fi




# RENAME FILES
# Regex warper.
function easy_rename
{
    if [[ $3 ]]; then
        rename "s/$1/$2/g" *$3*
    else
        rename "s/$1/$2/g" *
    fi
}
alias rename='easy_rename'



# FZF (FUZZY FINDER)
export FZF_HEIGHT=16
# If fzf is installed, source it, and configure it accordingly.
if [[ $FZF ]]; then

    # Pattern needed to trigger fuzzy shell completion when Tab is pressed.
    export FZF_COMPLETION_TRIGGER=""

    # Use `fd` as the finder, rather than `find`.
    #if [[ -e /usr/bin/fd ]]; then
        #export FZF_DEFAULT_COMMAND="fd --hidden --no-ignore --exclude \.git --max-depth 5"
    #elif [[ -e /usr/bin/fdfind ]]; then
        #export FZF_DEFAULT_COMMAND="fdfind --hidden --no-ignore --exclude \.git --max-depth 5"
    #fi

    # Default arguments passed each time `fzf` is launched.
    export FZF_DEFAULT_OPTS="--tiebreak=length,begin,end --multi --height=$FZF_HEIGHT --layout=reverse --info=inline"

    # Default options when `fzf` is launched in history mode
    export FZF_CTRL_R_OPTS="--sort"
fi



##### APT + FZF #####
# APT Upgrade.
function auto-apt-update
{
    sudo apt update
    sudo apt upgrade
    sudo apt autoremove --purge
    sudo apt autoclean
}
alias au='auto-apt-update'

# APT Install.
function auto-apt-install
{
    if [[ $1 ]]; then
        sudo apt update
        sudo apt upgrade
        sudo apt install $@
        sudo apt autoremove --purge
        sudo apt autoclean
    # If no argument was given, check if fzf is installed, and use it to select packages.
    elif [[ -e $FZF ]]; then
        sudo apt update
        PACKAGES=$(sudo apt list | fzf --multi | cut -f 1 -d '/')
        sudo apt upgrade
        sudo apt install $PACKAGES
        sudo apt autoremove --purge
        sudo apt autoclean
    fi
}
alias ai='auto-apt-install'

# APT Remove.
function auto-apt-remove
{
    if [[ $1 ]]; then
        sudo apt update
        sudo apt remove --purge $@
        sudo apt autoremove --purge
        sudo apt upgrade
        sudo apt autoremove --purge
        sudo apt autoclean
    # If no argument was given, check if fzf is installed, and use it to select packages.
    elif [[ -e $FZF ]]; then
        sudo apt update
        PACKAGES=$(sudo apt list --installed | fzf --multi | cut -f 1 -d '/')
        sudo apt remove --purge $PACKAGES
        sudo apt autoremove --purge
        sudo apt upgrade
        sudo apt autoremove --purge
        sudo apt autoclean
    fi
}
alias ar='auto-apt-remove'




##### GIT #####
# Status.
alias gs='git status'

# Clone.
if [[ -e /usr/bin/xclip ]]; then
    function shell-git-clone-clipboard
    {
        # If xclip is installed, use it clone the clipboard contents, so long as an arguent was not given.
        [[ $1 ]] && git clone $1 || git clone $(xclip -o)
    }
    alias gc='shell-git-clone-clipboard'
else
    alias gc='git clone'
fi

# Init.
function shell-git-init {
    git init
    # If an argument was given, then a remote will be added.
    [[ "$1" ]] && git remote add origin "$1"
}
alias gi='shell-git-init'

# Add.
function shell-git-add
{
    [[ $1 ]] && git add $@ || git add .
}
alias ga='shell-git-add'

# Reset.
alias gr='git reset'

# Commit.
function shell-git-commit {
    # Only the first argument from "$@" remains within the commit message brackets, hence to commit without using brackets...
    git commit -m "$1 $2 $3 $4 $5 $6 $7 $8 $9 $10 $11 $12 $13 $14 $15 $16 $17 $18 $19 $20 $21 $22 $23 $24 $25 $26 $27 $28 $29 $30 $31 $32 $33 $34 $35 $36 $37 $38 $39 $40 $41 $42 $43 $44 $45 $46 $47 $48 $49 $50"
}
alias gm='shell-git-commit'

# Push.
function shell-git-push {
    if [[ $2 ]]; then
        git push $1 $2
    elif [[ $1 ]]; then
        git push origin $1
    else
        git push origin master
    fi
}
alias gp='shell-git-push'

# Pull.
function shell-git-pull {
    if [[ $2 ]]; then
        git pull $1 $2
    elif [[ $1 ]]; then
        git pull origin $1
    else
        git pull origin master
    fi
}
alias gl='shell-git-pull'

# Switch branch.
function shell-git-switch-branch {
    # If a branch was not found, then it will be created.
    git switch $1 2>/dev/null || git switch -c $1
}
alias gb='shell-git-switch-branch'

# Delete branch.
alias gdb='git branch -d'




# Start a tmux session if not already in one.
if [[ $TERM != "screen" ]]; then
    tmux -f $CONFIG/tmux.conf
fi




# Finally, show a newcomer's welcome!
echo "


                                                )(|   |)(
        IMMERSIVE                                /     \
        SEMI-                         )(——————————————————————————)(
        AUTONOMOUS                           / /··········\ \
        AERIAL                              | |············| |
        COMMAND                              \_\··········/_/
        SYSTEM                                /(          )\


        ~ Welcome to ISAACS! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                COMMANDS

                F1     go to   <-| LEFT   TAB
                F2     go to   RIGHT |->  TAB
                F3     SPLIT-SCREEN (Vertically)
                F4     NEW TERMINAL TAB
                F7     SPLIT-SCREEN (Horizontally)
                F12    CLOSE THIS TAB

      Shift +   F1     move this tab to the   <#| LEFT
      Shift +   F2     move this tab to the   RIGHT |#>



                 ^
       Alt  +  < ⌄ >   NAVIGATE BETWEEN SPLITS CREATED WITH F3/F7

                 ^
Ctrl + Alt  +  < ⌄ >   RESIZE THE SPLITS CREATED WITH F3/F7

                 ^
     Shift  +  < ⌄ >   MOVE THE SPLITS CREATED WITH F3/F7



       Ctrl +   R      TERMINAL HISTORY (use this instead of up/down arrows!)

        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        $ au                UPDATE CURRENTLY INSTALLED PACKAGES (apt update)
        $ ai                smart menu to search and select PACKAGES TO INSTALL
        $ ar                smart menu to search and select PACKAGES TO REMOVE

        $ find <pattern>    FIND A FILE WHOSE NAME CONTAINS <pattern>

        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        *** See   ~/ISAACS-config/HOWTO-ROS.pdf   for a list of ROS commands. ***
"
