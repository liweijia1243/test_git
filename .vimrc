"
"__        __   _  _ _        __     ___                    
"\ \      / /__(_)(_|_) __ _  \ \   / (_)_ __ ___  _ __ ___ 
" \ \ /\ / / _ \ || | |/ _` |  \ \ / /| | '_ ` _ \| '__/ __|
"  \ V  V /  __/ || | | (_| |   \ V / | | | | | | | | | (__ 
"   \_/\_/ \___|_|/ |_|\__,_|    \_/  |_|_| |_| |_|_|  \___|
"               |__/                                        
"
"  李伟嘉自己的vimrc配置文件
set nocompatible
" Set <LEADER> as <SPACE>

let mapleader=" "
autocmd BufWritePost $MYVIMRC source $MYVIMRC "设置让vimrc变更立即生效

map <LEADER><up> :res +5<CR>
map <LEADER><down> :res -5<CR>
map <LEADER><left> :vertical resize-5<CR>
map <LEADER><right> :vertical resize+5<CR>

set bg=dark

set ts=4
set expandtab
set shiftwidth=4
set softtabstop=4
set smarttab

set number
set cursorline "加一条线
set cursorcolumn "高亮显示光标列
set wrap    "自动换行不会超过屏幕显示
set showcmd "显示输入字符
set wildmenu "tab补全vim命令 如source
set hlsearch "用/进行搜索时，高亮显示单词
set scrolloff=10
set ruler
set clipboard+=unnamedplus "设置vim与系统公用剪切板，可以方便使用“+y，以及”+p来实现夸vim与系统的剪切板共享
"配置大写Y用来复制内容至剪切板
noremap Y "+y 

set mouse=a
"set paste 
"set selection=exclusive    
"set selectmode=mouse,key  

"let &t_SI = "\<Esc>]50;CursorShape=1\x7" 
"let &t_SR = "\<Esc>]50;CursorShape=2\x7" 
"let &t_EI = "\<Esc>]50;CursorShape=0\x7" 

" split the screens to up (horizontal), down (horizontal), left (vertical),
" right (vertical)
map si :set nosplitbelow<CR>:split<CR>:set splitbelow<CR>
map sk :set splitbelow<CR>:split<CR>
map sj :set nosplitright<CR>:vsplit<CR>:set splitright<CR>
map sl :set splitright<CR>:vsplit<CR>
noremap s <nop>

"Use <space> + new arrow keys for moving the cursor around windows
map <LEADER>w <C-w>w
map <LEADER>i <C-w>k
map <LEADER>k <C-w>j
map <LEADER>j <C-w>h
map <LEADER>l <C-w>l

"我将默认的上下左右kjhl改为了ikjl，这样操作更方便快捷
"    ^
"    i
"<j      l >
"    k
"    ,
noremap ;  :
noremap j  h
noremap k  j
noremap i  k
noremap h  i

noremap H  I

noremap I  5k
noremap K  5j

map tu :tabe<CR>
" Move around tabs with th and tl
map tj :-tabnext<CR>
map tl :tabnext<CR>

exec "nohlsearch"
set incsearch
set ignorecase "搜索时忽略大小写
set smartcase 
"插件管理软件
filetype off 
set rtp+=~/.vim/bundle/Vundle.vim

call vundle#begin()

Plugin 'VundleVim/Vundle.vim'
Plugin 'ycm-core/YouCompleteMe'
Plugin 'preservim/nerdtree'
Plugin 'vim-airline/vim-airline'
Plugin 'vim-airline/vim-airline-themes'
Plugin 'connorholyday/vim-snazzy'
Plugin 'octol/vim-cpp-enhanced-highlight'
Plugin 'lilydjwg/fcitx.vim'
Plugin 'Xuyuanp/nerdtree-git-plugin'
Plugin 'fadein/vim-FIGlet'
Plugin 'junegunn/fzf', { 'do': 'fzf#install()' }
Plugin 'junegunn/fzf.vim'
Plugin 'mileszs/ack.vim'

call vundle#end()
filetype plugin indent on 
filetype plugin on
syntax on;
let g:airline_theme='simple'
let g:SnazzyTransparent = 1
colorscheme snazzy
" air-line
let g:airline_powerline_fonts = 1

if !exists('g:airline_symbols')
    let g:airline_symbols = {}
endif

" unicode symbols
let g:airline_left_sep = '»'
let g:airline_left_sep = '▶'
let g:airline_right_sep = '«'
let g:airline_right_sep = '◀'
let g:airline_symbols.linenr = '␊'
let g:airline_symbols.linenr = '␤'
let g:airline_symbols.linenr = '¶'
let g:airline_symbols.branch = '⎇'
let g:airline_symbols.paste = 'ρ'
let g:airline_symbols.paste = 'Þ'
let g:airline_symbols.paste = '∥'
let g:airline_symbols.whitespace = 'Ξ'
" airline symbols
let g:airline_left_sep = ''
let g:airline_left_alt_sep = ''
let g:airline_right_sep = ''
let g:airline_right_alt_sep = ''
let g:airline_symbols.branch = ''
let g:airline_symbols.readonly = ''
let g:airline_symbols.linenr = ''

"--------------------------------------------------------------------------
"设置NERDTree的快捷配置
"--------------------------------------------------------------------------
map tt :NERDTreeToggle<CR> "双击t，打开nerdtree
let NERDTreeMapOpenSplit="h"

"-------------------------------------------------------------------------
"设置nerdtreegit插件的显示模式
"------------------------------------------------------------------------

let g:NERDTreeIndicatorMapCustom = {
    \ "Modified"  : "✹",
    \ "Staged"    : "✚",
    \ "Untracked" : "✭",
    \ "Renamed"   : "➜",
    \ "Unmerged"  : "═",
    \ "Deleted"   : "✖",
    \ "Dirty"     : "✗",
    \ "Clean"     : "✔︎",
    \ 'Ignored'   : '☒',
    \ "Unknown"   : "?"
    \ }




""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"设置ycm的快捷配置
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
nnoremap gd :YcmCompleter GoToDefinitionElseDeclaration<CR>
nnoremap g/ :YcmCompleter GetDoc<CR>
nnoremap gt :YcmCompleter GetType<CR>
nnoremap gr :YcmCompleter GoToReferences<CR>
let g:ycm_autoclose_preview_window_after_completion=1
let g:ycm_autoclose_preview_window_after_insertion=1
let g:ycm_min_num_of_chars_for_completion = 2
let g:ycm_collect_identifiers_from_comments_and_strings = 1
let g:ycm_add_preview_to_completeopt = 1
let g:ycm_goto_buffer_command = 'split'
let g:ycm_key_list_select_completion = ['<TAB>', '<Down>']
let g:ycm_global_ycm_extra_conf = '~/.vim/bundle/YouCompleteMe/global_path/.ycm_extra_conf.py'
