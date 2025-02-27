return {
	"nvim-telescope/telescope.nvim",
	tag = "0.1.8",
	dependencies = { "nvim-lua/plenary.nvim" },
	config = function()
		local builtin = require("telescope.builtin")

		vim.keymap.set("n", "<leader>ff", function()
			builtin.find_files()
		end, {})
		vim.keymap.set("n", "<leader>fa", function()
			builtin.find_files({ hidden = true, no_ignore = true, no_ignore_parent = true })
		end, {})
		vim.keymap.set("n", "<leader>gf", builtin.git_files, {})
	end,
}
