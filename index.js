const express = require('express');
const fs = require('fs');
const path = require('path');
let marked; // Declare marked variable

// Dynamically import marked
import('marked').then(module => {
    marked = module.marked;
}).catch(err => {
    console.error('Failed to load marked library:', err);
    process.exit(1);
});

const app = express();
const port = 3000;

// Serve static files (like CSS if we add it later)
app.use(express.static('public'));

// Route for the home page or a specific chapter
app.get('/', (req, res) => {
    res.redirect('/chapter/1'); // Redirect to Chapter 1 by default
});

app.get('/chapter/:chapterNumber', (req, res) => {
    const chapterNumber = req.params.chapterNumber;
    const filePath = path.join(__dirname, `chapter_${chapterNumber}.md`);

    fs.readFile(filePath, 'utf8', (err, data) => {
        if (err) {
            if (err.code === 'ENOENT') {
                return res.status(404).send(`Chapter ${chapterNumber} not found.`);
            } else {
                console.error(`Error reading file for chapter ${chapterNumber}:`, err);
                return res.status(500).send('Error loading chapter.');
            }
        }

        const htmlContent = marked(data);

        // Basic HTML wrapper for consistent styling and navigation
        res.send(`
            <!DOCTYPE html>
            <html lang="en">
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <title>Chapter ${chapterNumber} - Physical AI & Humanoid Robotics</title>
                <style>
                    body { font-family: sans-serif; margin: 20px; line-height: 1.6; }
                    pre { background-color: #eee; padding: 10px; border-radius: 5px; overflow-x: auto; }
                    code { font-family: monospace; }
                    h1, h2, h3, h4 { color: #333; }
                    table { border-collapse: collapse; width: 100%; }
                    th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
                    th { background-color: #f2f2f2; }
                    .navigation { margin-bottom: 20px; }
                    .navigation a { margin-right: 15px; text-decoration: none; color: blue; }
                </style>
            </head>
            <body>
                <div class="navigation">
                    ${parseInt(chapterNumber) > 1 ? `<a href="/chapter/${parseInt(chapterNumber) - 1}">Previous Chapter</a>` : ''}
                    <a href="/">Home (Chapter 1)</a>
                    ${parseInt(chapterNumber) < 5 ? `<a href="/chapter/${parseInt(chapterNumber) + 1}">Next Chapter</a>` : ''}
                </div>
                ${htmlContent}
            </body>
            </html>
        `);
    });
});

app.listen(port, () => {
    console.log(`Textbook server running at http://localhost:${port}`);
    console.log('Navigate to http://localhost:3000/chapter/1 to view Chapter 1.');
});
