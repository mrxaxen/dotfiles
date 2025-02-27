vim.cmd("autocmd BufWinEnter *.py nmap <silent> <F5>:w<CR>:terminal python3 -m pdb '%:p'<CR>")
