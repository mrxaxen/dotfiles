require("xaxen.sets")
require("xaxen.remap")
require("xaxen.color")
require("xaxen.autocmd")

vim.loader.enable()
local version = vim.version

-- check if we have the latest stable version of nvim
local expected_ver = "0.9.5"
local ev = version.parse(expected_ver)
local actual_ver = version()

if version.cmp(ev, actual_ver) ~= 0 then
    local _ver = string.format("%s.%s.%s", actual_ver.major, actual_ver.minor, actual_ver.patch)
    local msg = string.format("Expect nvim %s, but got %s instead. Use at your own risk!", expected_ver, _ver)
    vim.api.nvim_err_writeln(msg)
end

local lazypath = vim.fn.stdpath("data") .. "/lazy/lazy.nvim"
if not vim.loop.fs_stat(lazypath) then
    vim.fn.system({
        "git",
        "clone",
        "--filter=blob:none",
        "https://github.com/folke/lazy.nvim.git",
        "--branch=stable", -- latest stable release
        lazypath,
    })
end
vim.opt.rtp:prepend(lazypath)

local opts = {}

local plugins = {
    "folke/which-key.nvim",
    {
        "folke/tokyonight.nvim",
        config = function(n)
            vim.cmd("colorscheme tokyonight")
            --vim.cmd("colorscheme tokyonight-night")
            --vim.cmd("colorscheme tokyonight-storm")
            --vim.cmd("colorscheme tokyonight-day")
            --vim.cmd("colorscheme tokyonight-moon")
        end
    },
    { "folke/neoconf.nvim", cmd = "Neoconf" },
    "folke/neodev.nvim",
    {
        "folke/trouble.nvim",
        dependencies = { "nvim-tree/nvim-web-devicons" },
        config = function ()
            vim.keymap.set("n", "<leader>tt", vim.cmd.TroubleToggle)
            vim.keymap.set("n", "<leader>tc", function() vim.cmd.TroubleToggle {"todo"} end)
        end
    },
    {
        "folke/todo-comments.nvim",
        dependencies = { "nvim-lua/plenary.nvim" },
        opts = {
            -- your configuration comes here
            -- or leave it empty to use the default settings
            -- refer to the configuration section below
        }
    },
    {
        "ThePrimeagen/harpoon",
        branch = "harpoon2",
        dependencies = { "nvim-lua/plenary.nvim" },
        config = function()
            local harpoon = require("harpoon")

            harpoon:setup()

            vim.keymap.set("n", "<leader>ha", function() harpoon:list():add() end)
            vim.keymap.set("n", "<C-h>", function() harpoon.ui:toggle_quick_menu(harpoon:list()) end)
            vim.keymap.set("n", "<C-j>", function() harpoon:list():select(1) end)
            vim.keymap.set("n", "<C-k>", function() harpoon:list():select(2) end)
            vim.keymap.set("n", "<C-l>", function() harpoon:list():select(3) end)
            vim.keymap.set("n", "<C-é>", function() harpoon:list():select(4) end)
            vim.keymap.set("n", "<C-S-P>", function() harpoon:list():prev() end)
            vim.keymap.set("n", "<C-S-N>", function() harpoon:list():next() end)
        end
        },
    {
        "nvim-telescope/telescope.nvim",
        tag = "0.1.6",
        dependencies = {
            "nvim-lua/plenary.nvim",
        },
        config = function ()
            local builtin = require("telescope.builtin")

            vim.keymap.set("n", "<leader>ff", function() builtin.find_files() end, {})
            vim.keymap.set("n", "<leader>fa", function() builtin.find_files({ hidden=true, no_ignore=true, no_ignore_parent=true }) end, {})
            vim.keymap.set("n", "<leader>gf", builtin.git_files, {})
        end
    },
    {
        "tpope/vim-fugitive",
        config = function()
            vim.keymap.set("n", "<leader>gs", vim.cmd.Git)
        end
    },
    {
        "mbbill/undotree",
        config = function()
            vim.keymap.set("n", "<leader>u", vim.cmd.UndotreeToggle)
        end
    },
    {
        "nvim-treesitter/nvim-treesitter",
        build = { ":TSUpdate" },
        config = {
            ensure_installed = {
                'lua', 'vim', 'vimdoc', 'c', 'java', 'go', 'zig', 'python', 'javascript'
            },
            auto_install = true,
            highlight = {
                enable = true,
                additional_vim_regex_highlighting = false,
            },
        },
    },
    --    "nvim-treesitter/playground", -- syntax tree
    {
        "VonHeikemen/lsp-zero.nvim",
        branch = "v3.x",
        config = false,
        init = function()
            vim.g.lsp_zero_extend_cmp = 0
            vim.g.lsp_zero_extend_lspconfig = 0
        end,
    },
    { "williamboman/mason.nvim",          config = true,  lazy = false },
    { "williamboman/mason-lspconfig.nvim" },
    {
        "hrsh7th/nvim-cmp",
        event = "InsertEnter",
        dependencies = {
            "L3MON4D3/LuaSnip",
            "hrsh7th/cmp-buffer",
            "hrsh7th/cmp-path",
            "hrsh7th/cmp-nvim-lsp",
            "hrsh7th/cmp-nvim-lua",
            "hrsh7th/cmp-cmdline",
            "saadparwaiz1/cmp_luasnip",
            "rafamadriz/friendly-snippets",
        },
        config = function()
            local lsp_zero = require("lsp-zero")
            lsp_zero.extend_cmp()

            local cmp = require("cmp")

            cmp.setup({
                formatting = lsp_zero.cmp_format({ details = true }),
                mapping = cmp.mapping.preset.insert({
                    ["<CR>"] = cmp.mapping.confirm({ select = true }),
                    ["<C-B>"] = cmp.mapping.complete(),
                    --["<C-n>"] = cmp.select_next_item({behavior = "select"}),
                    --["<C-p>"] = cmp.select_prev_item({behavior = "select"}),
                    ["<C-n"] = cmp.mapping(function ()
                        if cmp.visible then
                            cmp.select_next_item({ behavior = "select" })
                        else
                            cmp.complete()
                        end
                    end),
                    ["<C-p"] = cmp.mapping(function ()
                        if cmp.visible then
                            cmp.select_prev_item({ behavior = "select" })
                        else
                            cmp.complete()
                        end
                    end),
                    ["<C-u>"] = cmp.mapping.scroll_docs(-4),
                    ["<C-d>"] = cmp.mapping.scroll_docs(4),
                }),
            })
        end,
    },
    {
        "neovim/nvim-lspconfig",
        dependencies = {
            "williamboman/mason.nvim",
            "williamboman/mason-lspconfig.nvim",
        },
        config = function()
            local lsp_zero = require("lsp-zero")
            lsp_zero.extend_lspconfig()
            lsp_zero.extend_cmp()
            lsp_zero.on_attach(function(client, bufnr) lsp_zero.default_keymaps({ buffer = bufnr }) end)

            local lspconfig = require("lspconfig")
            local lsp_capabilities = require("cmp_nvim_lsp").default_capabilities()

            local mason_lspconfig = require("mason-lspconfig")

            vim.lsp.set_log_level("info")
            mason_lspconfig.setup({
                ensure_installed = {
                    "bashls", "jdtls", "lua_ls", "tsserver", "zls", "basedpyright", "pylsp"
                },
                handlers = {
                    lsp_zero.default_setup,
                    jdtls = function()
                        lspconfig.jdtls.setup({
                            root_dir = function()
                                return vim.fs.dirname(vim.fs.find({
                                            "pom.xml",
                                            "settings.gradle",
                                            "settings.gradle.kts",
                                            "build.xml"
                                        },
                                        { upward = true })[1])
                                    or vim.cmd("pwd")
                            end
                        })
                    end,
                    lua_ls = function()
                        lspconfig.lua_ls.setup({
                            capabities = lsp_capabilities,
                            settings = {
                                Lua = {
                                    runtime = {
                                        version = "LuaJIT"
                                    },
                                    diagnostics = {
                                        globals = { "vim" },
                                    },
                                    workspace = {
                                        library = { vim.env.VIMRUNTIME },
                                    },
                                }
                            }
                        })
                    end,
                    bashls = function()
                        lspconfig.bashls.setup({
                            root_dir = function()
                                return vim.cmd("pwd")
                            end
                        })
                    end,
                    pylsp = function ()
                        lspconfig.pylsp.setup({
                            settings = {
                                pylsp = {
                                    plugins = {
                                        pycodestyle = {
                                            enabled = false,
                                            maxLineLength = 100
                                        },
                                        flake8 = {
                                            enabled = true,
                                            maxLineLength = 100
                                        },
                                        pyflakes = {
                                            enabled = false,
                                        },
                                        jedi_completion = {
                                            enabled = false,
                                        },
                                    }
                                }
                            },
                        })
                    end,
                    basedpyright = function ()
                        lspconfig.basedpyright.setup({
                            capabities = capabilities,
                            settings = {
                                basedpyright = {
                                    analysis = {
                                        diagnosticMode = "workspace",
                                        typeCheckingMode = "standard",
                                    },
                                },
                            },
                        })
                    end,
                }
            })

            vim.api.nvim_create_autocmd('LspAttach', {
                group = vim.api.nvim_create_augroup('UserLspConfig', {}),
                callback = function(ev)
                    -- Enable completion triggered by <c-x><c-o>
                    --vim.bo[ev.buf].omnifunc = 'v:lua.vim.lsp.omnifunc'

                    -- Buffer local mappings.
                    -- See `:help vim.lsp.*` for documentation on any of the below functions
                    local opts = { buffer = ev.buf }
                    vim.keymap.set('n', 'gD', vim.lsp.buf.declaration, opts)
                    vim.keymap.set('n', 'gd', vim.lsp.buf.definition, opts)
                    vim.keymap.set('n', 'gr', vim.lsp.buf.references, opts)
                    vim.keymap.set('n', 'gi', vim.lsp.buf.implementation, opts)
                    -- vim.keymap.set('n', '<leader>th', vim.lsp.buf.typehierarchy, opts)
                    vim.keymap.set('n', 'K', vim.lsp.buf.hover, opts)
                    vim.keymap.set('n', '<C-S-k>', vim.lsp.buf.signature_help, opts)
                    vim.keymap.set('n', '<leader>wa', vim.lsp.buf.add_workspace_folder, opts)
                    vim.keymap.set('n', '<leader>wr', vim.lsp.buf.remove_workspace_folder, opts)
                    vim.keymap.set('n', '<leader>wl', function()
                        print(vim.inspect(vim.lsp.buf.list_workspace_folders()))
                    end, opts)
                    vim.keymap.set('n', '<leader>D', vim.lsp.buf.type_definition, opts)
                    vim.keymap.set('n', '<leader>rn', vim.lsp.buf.rename, opts)
                    vim.keymap.set('n', '<leader>fm', vim.lsp.buf.format, opts)
                    vim.keymap.set({ 'n', 'v' }, '<leader>ca', vim.lsp.buf.code_action, opts)
                    vim.keymap.set('n', '<leader>f', function()
                        vim.lsp.buf.format { async = true }
                    end, opts)
                end,
            })
        end
    },
    {
        "rshkarin/mason-nvim-lint",
        dependencies = { "mfussenegger/nvim-lint" },
        config = function ()
            require("mason-nvim-lint").setup({
                ensure_installed = { "mypy" }
            })
        end
    },
    {
        "tadachs/ros-nvim",
        dependencies = { "nvim-lua/plenary.nvim" },
        config = function ()
            require("ros-nvim").setup({only_workspace = true})
        end
    },
}

require("lazy").setup(plugins, opts)
