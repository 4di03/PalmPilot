# Reddit Outreach

Posts human-sounding discussion threads across niche subreddits.
Each post genuinely asks about the problem and mentions the landing page naturally.

## One-time setup (~5 minutes)

### 1. Create a Reddit account
Use a fresh account not tied to your personal identity.
Let it age a few days before posting if possible — new accounts sometimes get auto-filtered.

### 2. Create a Reddit API app
1. Log in → [reddit.com/prefs/apps](https://www.reddit.com/prefs/apps)
2. Click "create another app"
3. Type: **script**
4. Name: anything (e.g. "personal script")
5. Redirect URI: `http://localhost`
6. Copy the **client ID** (under the app name) and **client secret**

### 3. Fill in credentials
```bash
cp .env.example .env
# edit .env with your credentials
```

### 4. Install dependencies
```bash
pip install -r requirements.txt
```

---

## Usage

**Preview all posts (no submission):**
```bash
python post.py --dry-run
```

**Preview one niche:**
```bash
python post.py --dry-run --niche food-truck
```

**Post everything (15-minute gaps between posts):**
```bash
python post.py --url https://your-netlify-site.netlify.app
```

**Post one niche only:**
```bash
python post.py --niche food-truck --url https://your-netlify-site.netlify.app/food-truck
```

---

## Niches and subreddits

| Niche flag | Subreddits |
|---|---|
| `food-truck` | r/foodtrucks, r/smallbusiness |
| `nonprofit-grants` | r/nonprofit, r/fundraising |
| `machine-shop` | r/machinists, r/metalworking |
| `gun-shop` | r/FFL |
| `cleaning-business` | r/smallbusiness, r/Entrepreneur |
| `ada-compliance` | r/webdev, r/freelance |
| `auto-detailing` | r/AutoDetailing, r/mobiledetailing |
| `martial-arts` | r/martialarts, r/bjj |

---

## Notes

- Posts are spaced 15 minutes apart automatically to avoid spam filters
- Each post is written as a genuine community question, not an advertisement
- The landing page link appears naturally at the end as "building something for this"
- Do not post to the same subreddit more than once per week
