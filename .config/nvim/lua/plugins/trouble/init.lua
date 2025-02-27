return {
	"folke/trouble.nvim",
	opts = {},
	dependencies = { "nvim-tree/nvim-web-devicons" },
	keys = {
		{
			"<leader>tt",
			"<cmd>Trouble diagnostics toggle<cr>",
			desc = "Toggle location list",
		},
		{
			"<leader>tc",
			"<cmd>Trouble todo<cr>",
			desc = "Open TODOs",
		},
	},
}
