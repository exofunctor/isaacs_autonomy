git clone https://github.com/junegunn/fzf.git ~/ISAACS-config/fzf
cd ~/ISAACS-config
./fzf/install
mv $HOME/.bashrc $HOME/.bashrc_bkp
ln -sv ~/ISAACS-config/bash/bashrc $HOME/.bashrc
ln -sv ~/ISAACS-config/tmux.conf $HOME/.tmux.conf
